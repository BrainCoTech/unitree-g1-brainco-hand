#include <array>
#include <chrono>
#include <iostream>
#include <thread>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include <unitree/idl/go2/MotorCmds_.hpp>
#include <unitree/idl/go2/MotorStates_.hpp>

// brainco sdk
#include "dds/Publisher.h"
#include "dds/Subscription.h"

static const std::string kTopicArmSDK = "rt/arm_sdk";
static const std::string kTopicState = "rt/lowstate";
static const std::string kTopicHandCmdLeft = "rt/brainco/left/cmd";
static const std::string kTopicHandStateLeft = "rt/brainco/left/state";
static const std::string kTopicHandCmdRight = "rt/brainco/right/cmd";
static const std::string kTopicHandStateRight = "rt/brainco/right/state";


constexpr float kPi = 3.141592654;
constexpr float kPi_2 = 1.57079632;
#include <cmath>    // 必须
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

enum JointIndex {
    // Left leg
    kLeftHipPitch,
    kLeftHipRoll,
    kLeftHipYaw,
    kLeftKnee,
    kLeftAnkle,
    kLeftAnkleRoll,

    // Right leg
    kRightHipPitch,
    kRightHipRoll,
    kRightHipYaw,
    kRightKnee,
    kRightAnkle,
    kRightAnkleRoll,

    kWaistYaw,
    kWaistRoll,
    kWaistPitch,

    // Left arm
    kLeftShoulderPitch,
    kLeftShoulderRoll,
    kLeftShoulderYaw,
    kLeftElbow,
    kLeftWristRoll,
    kLeftWristPitch,
    kLeftWristYaw,
    // Right arm
    kRightShoulderPitch,
    kRightShoulderRoll,
    kRightShoulderYaw,
    kRightElbow,
    kRightWristRoll,
    kRightWristPitch,
    kRightWristYaw,

    kNotUsedJoint,
    kNotUsedJoint1,
    kNotUsedJoint2,
    kNotUsedJoint3,
    kNotUsedJoint4,
    kNotUsedJoint5
};

// 非阻塞键盘
int kbhit() {
    termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return ch;
    }
    return 0;
}

int main(int argc, char const *argv[]) {
  if (argc < 2) {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);

  unitree::robot::ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_>
      arm_sdk_publisher;
  unitree_hg::msg::dds_::LowCmd_ msg;

  arm_sdk_publisher.reset(
      new unitree::robot::ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(
          kTopicArmSDK));
  arm_sdk_publisher->InitChannel();

  unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_>
      low_state_subscriber;

  // create subscriber
  unitree_hg::msg::dds_::LowState_ state_msg;
  low_state_subscriber.reset(
      new unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(
          kTopicState));
  low_state_subscriber->InitChannel([&](const void *msg) {
        auto s = ( const unitree_hg::msg::dds_::LowState_* )msg;
        memcpy( &state_msg, s, sizeof( unitree_hg::msg::dds_::LowState_ ) );
  }, 1);

  std::array<JointIndex, 17> arm_joints = {
      JointIndex::kLeftShoulderPitch,  JointIndex::kLeftShoulderRoll,
      JointIndex::kLeftShoulderYaw,    JointIndex::kLeftElbow,
      JointIndex::kLeftWristRoll,       JointIndex::kLeftWristPitch,
      JointIndex::kLeftWristYaw,
      JointIndex::kRightShoulderPitch, JointIndex::kRightShoulderRoll,
      JointIndex::kRightShoulderYaw,   JointIndex::kRightElbow,
      JointIndex::kRightWristRoll,      JointIndex::kRightWristPitch,
      JointIndex::kRightWristYaw,
      JointIndex::kWaistYaw,
      JointIndex::kWaistRoll,
      JointIndex::kWaistPitch};

  float weight = 0.f;
  float weight_rate = 0.2f;

  float kp = 60.f;
  float kd = 1.5f;
  float dq = 0.f;
  float tau_ff = 0.f;

  float control_dt = 0.02f;
  float max_joint_velocity = 0.5f;

  float delta_weight = weight_rate * control_dt;
  float max_joint_delta = max_joint_velocity * control_dt;
  auto sleep_time =
      std::chrono::milliseconds(static_cast<int>(control_dt / 0.001f));

  std::array<float, 17> target{0, 0, 0, 0, 0, 0, 0,
                                  0, 0, 0, 0, 0, 0, 0,
                                  0, 0, 0};
    std::array<float, 17> init_pos{
        0.4, 0.15, 0., 0.8, 0., 0., 0.,
        0.4, -0.15, 0., 0.8, 0., 0., 0.,
        0., 0., 0.
    };

    bool stop_flag = false;
    bool reset_mode = false;
    int reset_step = 0;
    int reset_steps = 100; // 2秒
    std::array<float, 17> reset_start_pos;

  // --- 手指 ---
  // 左手
  auto left_hand_publisher = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorCmds_>>(kTopicHandCmdLeft);
  left_hand_publisher->msg_.cmds().resize(6);
  for(auto & finger : left_hand_publisher->msg_.cmds())
  {
      finger.dq() = 1.; // max speed
  }
  auto left_hand_subscriber = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>(kTopicHandStateLeft);
  left_hand_subscriber->wait_for_connection();

  // 右手
  auto right_hand_publisher = std::make_unique<unitree::robot::RealTimePublisher<unitree_go::msg::dds_::MotorCmds_>>(kTopicHandCmdRight);
  right_hand_publisher->msg_.cmds().resize(6);
  for(auto & finger : right_hand_publisher->msg_.cmds())
  {
      finger.dq() = 1.; // max speed
  }
  auto right_hand_subscriber = std::make_shared<unitree::robot::SubscriptionBase<unitree_go::msg::dds_::MotorStates_>>(kTopicHandStateRight);
  right_hand_subscriber->wait_for_connection();

  // wait for init
  std::cout << "Press ENTER to init arms ...";
  std::cin.get();

  // get current joint position
  std::array<float, 17> current_jpos{};
  std::cout<<"Current joint position: ";
  for (int i = 0; i < arm_joints.size(); ++i) {
	current_jpos.at(i) = state_msg.motor_state().at(arm_joints.at(i)).q();
	std::cout << current_jpos.at(i) << " ";
  }
  std::cout << std::endl;

  // set init pos
  std::cout << "Initailizing arms ...";
  float init_time = 2.0f;
  int init_time_steps = static_cast<int>(init_time / control_dt);

  for (int i = 0; i < init_time_steps; ++i) {
    // increase weight
    weight = 1.0;
    msg.motor_cmd().at(JointIndex::kNotUsedJoint).q(weight);
    float phase = 1.0 * i / init_time_steps;
    std::cout << "Phase: " << phase << std::endl;

    if (!stop_flag) {
        // set control joints
        for (int j = 0; j < target.size(); ++j) {
        msg.motor_cmd().at(arm_joints.at(j)).q(target.at(j) * phase + current_jpos.at(j) * (1 - phase));
        msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
        msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
        msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
        msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
        }

        // send dds msg
        arm_sdk_publisher->Write(msg);
    }

    // sleep
    std::this_thread::sleep_for(sleep_time);
  }

  // start control
  std::cout << "Start arm movement." << std::endl;
std::cout << "Press 'r' to reset." << std::endl;
  
  // 手臂运动参数
  float amplitude = 0.1f;   // 摆动幅度（弧度）
  float frequency = 0.5f;   // 摆动频率 Hz

  // current joint positions (初始化为初始位置)
  std::array<float, 17> current_jpos_des = target;

  int step = 0;
  while (true) {
        // 键盘检测
        int key = kbhit();
        if ((key == 'r' || key == 'R') && !reset_mode) {
            std::cout << "STOP TRIGGERED!" << std::endl;
            stop_flag = true;

            // 用当前控制值作为复位起点（关键优化）
            reset_start_pos = current_jpos_des;

            // 进入复位模式
            reset_mode = true;
            reset_step = 0;
            step = 0;

            // 手指立即复位（张开）
            std::array<float,6> reset_pos = {0,0,0,0,0,0};

            for(int i = 0; i < 6; ++i)
                left_hand_publisher->msg_.cmds()[i].q() = reset_pos[i];
            left_hand_publisher->unlockAndPublish();

            for(int i = 0; i < 6; ++i)
                right_hand_publisher->msg_.cmds()[i].q() = reset_pos[i];
            right_hand_publisher->unlockAndPublish();
        }

        // ================== 手臂复位 ==================
        if (reset_mode) {

            std::array<float,6> reset_pos = {0,0,0,0,0,0};

            for(int i = 0; i < 6; ++i)
                left_hand_publisher->msg_.cmds()[i].q() = reset_pos[i];
            left_hand_publisher->unlockAndPublish();

            for(int i = 0; i < 6; ++i)
                right_hand_publisher->msg_.cmds()[i].q() = reset_pos[i];
            right_hand_publisher->unlockAndPublish();


            float phase = std::min(1.0f, (float)reset_step / reset_steps);

            for (int j = 0; j < 17; ++j) {

                float q = reset_start_pos[j] * (1 - phase) + init_pos[j] * phase;

                msg.motor_cmd().at(arm_joints.at(j)).q(q);
                msg.motor_cmd().at(arm_joints.at(j)).dq(0);
                msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
                msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
                msg.motor_cmd().at(arm_joints.at(j)).tau(0);
            }

            arm_sdk_publisher->Write(msg);

            reset_step++;

            if (reset_step >= reset_steps) {
                reset_mode = false;
                stop_flag = false;

                // 同步当前位置为新起点（避免跳变）
                current_jpos_des = init_pos;
                target = init_pos;

                std::cout << "ARM RESET DONE. Exiting program." << std::endl;
                return 0;  // 直接退出程序
            }
        }

        // ================== 正常控制 ==================
        else if (!stop_flag) {

            float t = step * control_dt;

            for (int j = 0; j < target.size(); ++j) {

                if (j == 5 || j == 6 || j == 12 || j == 13) {
                    current_jpos_des.at(j) = target.at(j) + amplitude * std::sin(2 * M_PI * frequency * t + j);
                } else {
                    current_jpos_des.at(j) = target.at(j);
                }

                msg.motor_cmd().at(arm_joints.at(j)).q(current_jpos_des.at(j));
                msg.motor_cmd().at(arm_joints.at(j)).dq(dq);
                msg.motor_cmd().at(arm_joints.at(j)).kp(kp);
                msg.motor_cmd().at(arm_joints.at(j)).kd(kd);
                msg.motor_cmd().at(arm_joints.at(j)).tau(tau_ff);
            }

            arm_sdk_publisher->Write(msg);

            // 手指周期动作
            int hand_cycle = step % 200;
            std::array<float,6> hand_pos;

            if(hand_cycle < 100) hand_pos = {0,0,0,0,0,0};
            else if(hand_cycle < 120) hand_pos = {0,1,1,1,1,1};
            else hand_pos = {1,1,1,1,1,1};

            for(int i=0;i<6;++i) left_hand_publisher->msg_.cmds()[i].q() = hand_pos[i];
            left_hand_publisher->unlockAndPublish();

            for(int i=0;i<6;++i) right_hand_publisher->msg_.cmds()[i].q() = hand_pos[i];
            right_hand_publisher->unlockAndPublish();
            
        }

        // sleep 等待下一个控制周期
        std::this_thread::sleep_for(sleep_time);
        step++;
        
  }

  std::cout << "Done!" << std::endl;

  return 0;
}
