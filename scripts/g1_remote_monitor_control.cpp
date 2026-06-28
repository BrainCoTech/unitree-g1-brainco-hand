#include <arpa/inet.h>
#include <array>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <cerrno>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <shared_mutex>
#include <string>
#include <sys/socket.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <vector>

#include "gamepad.hpp"

#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/robot/g1/audio/g1_audio_client.hpp>
#include <unitree/robot/g1/loco/g1_loco_client.hpp>

static const std::string HG_STATE_TOPIC = "rt/lowstate";

enum class Key {
  NONE = 0,
  SELECT,
  START,
  L1,
  R1,
  L2,
  R2,
  F1,
  F2,
  A,
  B,
  X,
  Y,
  UP,
  RIGHT,
  DOWN,
  LEFT,
};

struct KeyCombo {
  std::array<Key, 3> keys{};
};

struct SmachBinding {
  KeyCombo combo;
  const char* cmd;
  const char* handside;
  const char* extra;
  bool stop_managed_programs_after_success;
};

struct FsmBinding {
  KeyCombo combo;
};

struct LaunchBinding {
  KeyCombo combo;
  const char* conda_init;
  const char* conda_env;
  const char* workdir;
  int watched_command_index;
  const char* ready_phrase;
  std::array<const char*, 2> commands;
};

struct TimedInternalCommand {
  int delay_ms;
  const char* cmd;
  const char* handside;
  const char* extra;
};

static constexpr std::array<SmachBinding, 8> kActionBindings{{
    {{{Key::F1, Key::Y}}, "a1_Hello", "right", "", false},
    {{{Key::F1, Key::B}}, "a2_Like", "both", "", false},
    {{{Key::F1, Key::X}}, "a3_Rock-Paper-Scissors", "right", "", false},
    {{{Key::F1, Key::A}}, "a4_Handshake", "right", "", false},
    {{{Key::F2, Key::DOWN}}, "a6_ArmDown", "both", "", false},
    {{{Key::F2, Key::UP}}, "a7_ArmUP", "both", "", false},
    {{{Key::F2, Key::LEFT}}, "a8_HandClose", "both", "", false},
    {{{Key::F2, Key::RIGHT}}, "a9_HandOpen", "both", "", false},
}};

static constexpr std::array<SmachBinding, 1> kInternalBindings{{
    {{{Key::L2, Key::B}}, "smach_s", "", "", true},
}};

static constexpr FsmBinding kFsmBinding{{Key::SELECT, Key::F2}};

static constexpr LaunchBinding kLaunchBinding{
    {{Key::L2, Key::START}},
    "source /home/unitree/miniforge3/etc/profile.d/conda.sh",
    "g1brainco",
    "~/unitree-g1-brainco-hand/brainco_ws",
    1,
    "Agents started",
    {"./launch/launch_robot.sh", "./launch/launch_trans.sh agent"}};

static constexpr std::array<TimedInternalCommand, 2> kPostAgentReadyCommands{{
    {1000, "smach_f", "", ""},
    {2000, "smach_a", "", ""},
}};

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

template <typename T>
class DataBuffer {
 public:
  void SetData(const T& new_data) {
    std::unique_lock<std::shared_mutex> lock(mutex_);
    data_ = std::make_shared<T>(new_data);
  }

  std::shared_ptr<const T> GetData() const {
    std::shared_lock<std::shared_mutex> lock(mutex_);
    return data_ ? data_ : nullptr;
  }

 private:
  mutable std::shared_mutex mutex_;
  std::shared_ptr<T> data_;
};

struct RemoteState {
  uint8_t raw[40] = {};
  Gamepad gamepad;
};

inline uint32_t Crc32Core(uint32_t* ptr, uint32_t len) {
  uint32_t xbit = 0;
  uint32_t data = 0;
  uint32_t crc32 = 0xFFFFFFFF;
  const uint32_t polynomial = 0x04c11db7;
  for (uint32_t i = 0; i < len; i++) {
    xbit = 1U << 31;
    data = ptr[i];
    for (uint32_t bits = 0; bits < 32; bits++) {
      if (crc32 & 0x80000000) {
        crc32 <<= 1;
        crc32 ^= polynomial;
      } else {
        crc32 <<= 1;
      }
      if (data & xbit) {
        crc32 ^= polynomial;
      }
      xbit >>= 1;
    }
  }
  return crc32;
}

class G1RemoteMonitor {
 public:
  explicit G1RemoteMonitor(const std::string& network_interface)
      : previous_pressed_mask_(0),
        fsm_query_latched_(false),
        launch_query_latched_(false) {
    ChannelFactory::Instance()->Init(0, network_interface);
    audio_client_ = std::make_shared<unitree::robot::g1::AudioClient>();
    audio_client_->Init();
    audio_client_->SetTimeout(10.0f);
    loco_client_ = std::make_shared<unitree::robot::g1::LocoClient>();
    loco_client_->Init();
    loco_client_->SetTimeout(10.0f);
    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(
        std::bind(&G1RemoteMonitor::LowStateHandler, this, std::placeholders::_1),
        1);
  }

 private:
  void LowStateHandler(const void* message) {
    const LowState_ low_state = *(const LowState_*)message;
    if (low_state.crc() !=
        Crc32Core((uint32_t*)&low_state, (sizeof(LowState_) >> 2) - 1)) {
      std::cout << "[ERROR] CRC Error" << std::endl;
      return;
    }

    RemoteState remote_state;
    REMOTE_DATA_RX rx{};
    std::memcpy(rx.buff, &low_state.wireless_remote()[0], sizeof(rx.buff));
    std::memcpy(remote_state.raw, rx.buff, sizeof(remote_state.raw));
    gamepad_.update(rx.RF_RX);
    remote_state.gamepad = gamepad_;
    remote_state_buffer_.SetData(remote_state);
    const uint32_t current_pressed_mask = PressedMask();
    PrintNewlyPressedKeys(current_pressed_mask);
    previous_pressed_mask_ = current_pressed_mask;

    const bool launch_combo_pressed = IsLaunchBindingActive();
    if (launch_combo_pressed && !launch_query_latched_) {
      LaunchManagedPrograms();
    }
    launch_query_latched_ = launch_combo_pressed;

    for (size_t i = 0; i < kActionBindings.size(); ++i) {
      const bool active = IsBindingActive(kActionBindings[i]);
      if (active && !action_binding_latched_[i]) {
        TriggerSmachCommand(kActionBindings[i], 46000);
      }
      action_binding_latched_[i] = active;
    }

    for (size_t i = 0; i < kInternalBindings.size(); ++i) {
      const bool active = IsBindingActive(kInternalBindings[i]);
      if (active && !internal_binding_latched_[i]) {
        if (TriggerSmachCommand(kInternalBindings[i], 43210) &&
            kInternalBindings[i].stop_managed_programs_after_success) {
          StopManagedPrograms();
          Speak("程序已终止");
        }
      }
      internal_binding_latched_[i] = active;
    }

    const bool fsm_combo_pressed = IsFsmBindingActive();
    if (fsm_combo_pressed && !fsm_query_latched_) {
      PrintFsmId();
    }
    fsm_query_latched_ = fsm_combo_pressed;

  }

  bool IsKeyPressed(Key key) const {
    switch (key) {
      case Key::SELECT:
        return gamepad_.select.pressed;
      case Key::START:
        return gamepad_.start.pressed;
      case Key::L1:
        return gamepad_.L1.pressed;
      case Key::R1:
        return gamepad_.R1.pressed;
      case Key::L2:
        return gamepad_.L2.pressed;
      case Key::R2:
        return gamepad_.R2.pressed;
      case Key::F1:
        return gamepad_.F1.pressed;
      case Key::F2:
        return gamepad_.F2.pressed;
      case Key::A:
        return gamepad_.A.pressed;
      case Key::B:
        return gamepad_.B.pressed;
      case Key::X:
        return gamepad_.X.pressed;
      case Key::Y:
        return gamepad_.Y.pressed;
      case Key::UP:
        return gamepad_.up.pressed;
      case Key::RIGHT:
        return gamepad_.right.pressed;
      case Key::DOWN:
        return gamepad_.down.pressed;
      case Key::LEFT:
        return gamepad_.left.pressed;
      case Key::NONE:
      default:
        return false;
    }
  }

  const char* KeyName(Key key) const {
    switch (key) {
      case Key::SELECT:
        return "SELECT";
      case Key::START:
        return "START";
      case Key::L1:
        return "L1";
      case Key::R1:
        return "R1";
      case Key::L2:
        return "L2";
      case Key::R2:
        return "R2";
      case Key::F1:
        return "F1";
      case Key::F2:
        return "F2";
      case Key::A:
        return "A";
      case Key::B:
        return "B";
      case Key::X:
        return "X";
      case Key::Y:
        return "Y";
      case Key::UP:
        return "UP";
      case Key::RIGHT:
        return "RIGHT";
      case Key::DOWN:
        return "DOWN";
      case Key::LEFT:
        return "LEFT";
      case Key::NONE:
      default:
        return "NONE";
    }
  }

  uint32_t ComboMask(const KeyCombo& combo) const {
    uint32_t mask = 0;
    for (const Key key : combo.keys) {
      if (key == Key::NONE) {
        continue;
      }
      mask |= (1U << static_cast<uint32_t>(key));
    }
    return mask;
  }

  uint32_t PressedMask() const {
    uint32_t mask = 0;
    constexpr std::array<Key, 16> kSupportedKeys{
        Key::SELECT, Key::START, Key::L1,    Key::R1,    Key::L2,   Key::R2,
        Key::F1,     Key::F2,    Key::A,     Key::B,     Key::X,    Key::Y,
        Key::UP,     Key::RIGHT, Key::DOWN,  Key::LEFT};
    for (const Key key : kSupportedKeys) {
      if (IsKeyPressed(key)) {
        mask |= (1U << static_cast<uint32_t>(key));
      }
    }
    return mask;
  }

  void PrintNewlyPressedKeys(uint32_t current_pressed_mask) const {
    const uint32_t newly_pressed_mask =
        current_pressed_mask & ~previous_pressed_mask_;
    if (newly_pressed_mask == 0) {
      return;
    }

    constexpr std::array<Key, 16> kSupportedKeys{
        Key::SELECT, Key::START, Key::L1,    Key::R1,    Key::L2,   Key::R2,
        Key::F1,     Key::F2,    Key::A,     Key::B,     Key::X,    Key::Y,
        Key::UP,     Key::RIGHT, Key::DOWN,  Key::LEFT};

    std::cout << "[" << CurrentTimeString() << "] Pressed:";
    for (const Key key : kSupportedKeys) {
      const uint32_t bit = 1U << static_cast<uint32_t>(key);
      if (newly_pressed_mask & bit) {
        std::cout << " " << KeyName(key);
      }
    }
    std::cout << std::endl;
  }

  bool IsComboActive(const KeyCombo& combo) const {
    return PressedMask() == ComboMask(combo);
  }

  bool IsBindingActive(const SmachBinding& binding) const {
    return IsComboActive(binding.combo);
  }

  bool IsFsmBindingActive() const {
    return IsComboActive(kFsmBinding.combo);
  }

  bool IsLaunchBindingActive() const {
    return IsComboActive(kLaunchBinding.combo);
  }

  void PrintFsmId() {
    int fsm_id = -1;
    if (!loco_client_) {
      std::cout << "fsm_id: <client not initialized>" << std::endl;
      return;
    }

    const int32_t ret = loco_client_->GetFsmId(fsm_id);
    if (ret == 0) {
      std::cout << "fsm_id: " << fsm_id << std::endl;
    } else {
      std::cout << "fsm_id: <error, ret=" << ret << ">" << std::endl;
    }
  }

  void Speak(const std::string& text, int8_t speaker = 0) {
    if (!audio_client_) {
      std::cout << "[" << CurrentTimeString()
                << "] Audio client not initialized" << std::endl;
      return;
    }

    const int32_t ret = audio_client_->TtsMaker(text, speaker);
    std::cout << "[" << CurrentTimeString() << "] TTS ret=" << ret
              << " text=" << text << std::endl;
  }

  std::string BuildLaunchShellCommand(const char* command) const {
    std::ostringstream oss;
    oss << kLaunchBinding.conda_init << " && "
        << "conda activate " << kLaunchBinding.conda_env << " && "
        << "cd " << kLaunchBinding.workdir << " && "
        << "exec " << command;
    return oss.str();
  }

  std::string BuildSmachExecRequest(const std::string& req_id,
                                    const char* cmd,
                                    const char* handside,
                                    const char* extra) const {
    std::ostringstream oss;
    oss << "{\"type\":\"exec\",\"req_id\":\"" << req_id
        << "\",\"cmd\":\"" << cmd << "\",\"args\":{\"handside\":\""
        << handside << "\",\"extra\":\"" << extra << "\"}}\n";
    return oss.str();
  }

  std::string MakeRequestId() const {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    const auto us =
        std::chrono::duration_cast<std::chrono::microseconds>(now).count();
    return std::to_string(us);
  }

  std::string CurrentTimeString() const {
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
    localtime_r(&time, &local_tm);

    std::ostringstream oss;
    oss << std::put_time(&local_tm, "%H:%M:%S");
    return oss.str();
  }

  void RunPostAgentReadyCommands() {
    for (const auto& command : kPostAgentReadyCommands) {
      std::this_thread::sleep_for(std::chrono::milliseconds(command.delay_ms));
      TriggerSmachCommand(command.cmd, command.handside, command.extra, 43210);
    }
    Speak("程序已准备");
  }

  void MonitorLaunchOutput(int read_fd, const char* command_label) {
    FILE* stream = fdopen(read_fd, "r");
    if (!stream) {
      close(read_fd);
      std::cout << "[" << CurrentTimeString()
                << "] Failed to monitor output for " << command_label
                << std::endl;
      return;
    }

    char buffer[1024];
    bool post_ready_started = false;
    while (fgets(buffer, sizeof(buffer), stream)) {
      std::string line(buffer);
      if (!line.empty() && line.back() == '\n') {
        line.pop_back();
      }

      std::cout << "[" << CurrentTimeString() << "] [" << command_label
                << "] " << line << std::endl;

      if (!post_ready_started &&
          line.find(kLaunchBinding.ready_phrase) != std::string::npos) {
        post_ready_started = true;
        std::thread(&G1RemoteMonitor::RunPostAgentReadyCommands, this).detach();
      }
    }

    fclose(stream);
  }

  void LaunchManagedPrograms() {
    ReapManagedPrograms();
    if (!managed_process_pids_.empty()) {
      std::cout << "[" << CurrentTimeString()
                << "] Launch skipped: managed programs still running"
                << std::endl;
      return;
    }

    bool launched_any = false;
    for (size_t i = 0; i < kLaunchBinding.commands.size(); ++i) {
      const char* command = kLaunchBinding.commands[i];
      const std::string shell_command = BuildLaunchShellCommand(command);
      int pipefd[2] = {-1, -1};
      const bool monitor_output =
          static_cast<int>(i) == kLaunchBinding.watched_command_index;
      if (monitor_output && pipe(pipefd) != 0) {
        std::cout << "[" << CurrentTimeString()
                  << "] Failed to create pipe for " << command << std::endl;
        continue;
      }

      const pid_t pid = fork();
      if (pid == 0) {
        setsid();
        if (monitor_output) {
          close(pipefd[0]);
          dup2(pipefd[1], STDOUT_FILENO);
          dup2(pipefd[1], STDERR_FILENO);
          close(pipefd[1]);
        }
        execl("/bin/bash", "bash", "-lc", shell_command.c_str(),
              static_cast<char*>(nullptr));
        _exit(127);
      }

      if (monitor_output) {
        close(pipefd[1]);
      }

      if (pid < 0) {
        std::cout << "[" << CurrentTimeString() << "] Failed to launch: "
                  << command << std::endl;
        if (monitor_output) {
          close(pipefd[0]);
        }
        continue;
      }

      managed_process_pids_.push_back(pid);
      launched_any = true;
      std::cout << "[" << CurrentTimeString() << "] Launched pid=" << pid
                << " cmd=" << command << std::endl;

      if (monitor_output) {
        std::thread(&G1RemoteMonitor::MonitorLaunchOutput, this, pipefd[0],
                    command)
            .detach();
      }
    }

    if (launched_any) {
      Speak("程序启动中");
    }
  }

  void ReapManagedPrograms() {
    std::vector<pid_t> alive;
    for (pid_t pid : managed_process_pids_) {
      int status = 0;
      const pid_t result = waitpid(pid, &status, WNOHANG);
      if (result == 0) {
        alive.push_back(pid);
      }
    }
    managed_process_pids_.swap(alive);
  }

  void StopManagedPrograms() {
    ReapManagedPrograms();
    if (managed_process_pids_.empty()) {
      std::cout << "[" << CurrentTimeString()
                << "] No managed programs to stop" << std::endl;
      return;
    }

    for (pid_t pid : managed_process_pids_) {
      kill(-pid, SIGTERM);
    }

    usleep(500000);

    for (pid_t pid : managed_process_pids_) {
      int status = 0;
      if (waitpid(pid, &status, WNOHANG) == 0) {
        kill(-pid, SIGKILL);
        waitpid(pid, &status, 0);
      }
    }

    managed_process_pids_.clear();
    std::cout << "[" << CurrentTimeString()
              << "] Managed programs stopped" << std::endl;
  }

  bool TriggerSmachCommand(const char* cmd, const char* handside,
                           const char* extra, int port) {
    const std::string req_id = MakeRequestId();
    std::cout << "[" << CurrentTimeString() << "] [" << port << "] Executing "
              << cmd << " handside=" << handside << " extra='" << extra << "'"
              << std::endl;

    const int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
      std::cout << "[" << CurrentTimeString()
                << "] [" << port << "] Response: socket create failed"
                << std::endl;
      return false;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, "127.0.0.1", &server_addr.sin_addr) != 1) {
      std::cout << "[" << CurrentTimeString()
                << "] [" << port << "] Response: invalid server address"
                << std::endl;
      close(sock);
      return false;
    }

    if (connect(sock, reinterpret_cast<sockaddr*>(&server_addr),
                sizeof(server_addr)) < 0) {
      std::cout << "[" << CurrentTimeString()
                << "] [" << port << "] Response: connect failed" << std::endl;
      close(sock);
      return false;
    }

    const std::string request = BuildSmachExecRequest(req_id, cmd, handside, extra);
    const ssize_t sent = send(sock, request.c_str(), request.size(), 0);
    if (sent < 0 || static_cast<size_t>(sent) != request.size()) {
      std::cout << "[" << CurrentTimeString()
                << "] [" << port << "] Response: send failed" << std::endl;
      close(sock);
      return false;
    }

    std::string response;
    char buffer[1024];
    while (true) {
      const ssize_t bytes = recv(sock, buffer, sizeof(buffer), 0);
      if (bytes <= 0) {
        break;
      }
      response.append(buffer, buffer + bytes);
      if (response.find('\n') != std::string::npos) {
        break;
      }
    }
    close(sock);

    if (response.empty()) {
      std::cout << "[" << CurrentTimeString()
                << "] [" << port << "] Response: <empty>" << std::endl;
      return false;
    }

    const size_t newline = response.find('\n');
    if (newline != std::string::npos) {
      response.erase(newline);
    }

    std::cout << "[" << CurrentTimeString() << "] [" << port << "] Response: "
              << response << std::endl;
    return response.find("\"ok\": true") != std::string::npos ||
           response.find("\"ok\":true") != std::string::npos;
  }

  bool TriggerSmachCommand(const SmachBinding& binding, int port) {
    return TriggerSmachCommand(binding.cmd, binding.handside, binding.extra,
                               port);
  }

  uint32_t previous_pressed_mask_;
  bool fsm_query_latched_;
  bool launch_query_latched_;
  std::array<bool, kActionBindings.size()> action_binding_latched_{};
  std::array<bool, kInternalBindings.size()> internal_binding_latched_{};
  DataBuffer<RemoteState> remote_state_buffer_;
  Gamepad gamepad_;
  std::vector<pid_t> managed_process_pids_;
  std::shared_ptr<unitree::robot::g1::AudioClient> audio_client_;
  std::shared_ptr<unitree::robot::g1::LocoClient> loco_client_;
  ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
};

int main(int argc, char const* argv[]) {
  if (argc < 2) {
    std::cout << "Usage: g1_remote_monitor network_interface" << std::endl;
    return 0;
  }

  std::cout << "Program Start." << std::endl;
  G1RemoteMonitor monitor(argv[1]);
  while (true) {
    sleep(10);
  }
  return 0;
}
