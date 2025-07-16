#include "ros2_stark_controller/stark_node.hpp"

StarkNode::StarkNode() : Node("stark_node") {
    // Declare and Get parameters
    port_l = this->declare_parameter<std::string>("port_l", "/dev/ttyUSB0");
    port_r = this->declare_parameter<std::string>("port_r", "/dev/ttyUSB1");
    baudrate_ = this->declare_parameter<int>("baudrate", 115200);
    slave_id_l = this->declare_parameter<int>("slave_id_l", 1);
    slave_id_r = this->declare_parameter<int>("slave_id_r", 1);
    fw_type_ = static_cast<StarkFirmwareType>(this->declare_parameter<int>("firmware_type", 2));
    protocol_type_ = static_cast<StarkProtocolType>(this->declare_parameter<int>("protocol_type", 1));
    log_level_ = static_cast<LogLevel>(this->declare_parameter<int>("log_level", 2));

    RCLCPP_INFO(this->get_logger(), "Port: %s, Baudrate: %d, slave_id: %d, firmware_type: %d, protocol_type: %d, log_level: %d",
                port_l.c_str(), baudrate_, slave_id_l, fw_type_, protocol_type_, log_level_);
    RCLCPP_INFO(this->get_logger(), "Port: %s, Baudrate: %d, slave_id: %d, firmware_type: %d, protocol_type: %d, log_level: %d",
                port_r.c_str(), baudrate_, slave_id_r, fw_type_, protocol_type_, log_level_);

    // Initialize Stark SDK
    init_cfg(fw_type_, protocol_type_, log_level_);
    // list_available_ports();
    if (!initialize_modbus()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to initialize Modbus");
        throw std::runtime_error("Modbus initialization failed");
    }
    else {
        RCLCPP_INFO(this->get_logger(), "Waiting for joint cmd ...");
    }

    // Initialize publishers
    motor_status_pub_ = create_publisher<ros2_stark_interfaces::msg::MotorStatus>("motor_status", 10);
    turbo_mode_pub_ = create_publisher<std_msgs::msg::Bool>("turbo_mode", 10);
    voltage_pub_ = create_publisher<std_msgs::msg::UInt16>("voltage", 10);


    joint_cmd_sub_left = create_subscription<sensor_msgs::msg::JointState>(
        "joint_commands_left", 10, std::bind(&StarkNode::joint_cmd_callback_left, this, std::placeholders::_1));
    joint_cmd_sub_right = create_subscription<sensor_msgs::msg::JointState>(
        "joint_commands_right", 10, std::bind(&StarkNode::joint_cmd_callback_right, this, std::placeholders::_1));
    // Initialize timer
    timer_ = create_wall_timer(
        std::chrono::milliseconds(50),
        std::bind(&StarkNode::timer_callback, this));
    
    // 设备电量等信息，不需要频繁更新    
    info_timer_ = create_wall_timer(
        std::chrono::seconds(30),
        std::bind(&StarkNode::info_timer_callback, this));
}

StarkNode::~StarkNode() {
    if (handle_l) {
        modbus_close(handle_l);
    }
    if (handle_r) {
        modbus_close(handle_r);
    }
}

bool StarkNode::initialize_modbus() {
    handle_l = modbus_open(port_l.c_str(), baudrate_, slave_id_l);
    handle_r = modbus_open(port_r.c_str(), baudrate_, slave_id_r);
    if (!handle_l) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open Modbus on %s", port_l.c_str());
        return false;
    }
    if (!handle_r) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open Modbus on %s", port_r.c_str());
        return false;
    }
    return true;
}

void StarkNode::timer_callback() {
    publish_motor_status();
}

void StarkNode::info_timer_callback() {
    publish_device_status();
}

void StarkNode::publish_joint_state() {
    auto motor_status = modbus_get_motor_status(handle_l, slave_id_l);
    auto motor_status_r = modbus_get_motor_status(handle_r, slave_id_r);
    if (!motor_status) {
        RCLCPP_WARN(this->get_logger(), "Failed to get LEFT motor status");
        return;
    }
    if (!motor_status_r) {
        RCLCPP_WARN(this->get_logger(), "Failed to get RIGHT motor status");
        return;
    }

    auto msg = sensor_msgs::msg::JointState();
    msg.header.stamp = this->now();
    msg.name = {"thumb", "thumb_aux", "index", "middle", "ring", "pinky"};
    for (int i = 0; i < 6; i++) {
        msg.position.push_back(motor_status->positions[i] / 100.0);  // Normalize to 0-1
        msg.velocity.push_back(motor_status->speeds[i] / 100.0);     // Normalize to -1-1
        msg.effort.push_back(motor_status->currents[i]);
    }

    joint_state_pub_->publish(msg);
    free_motor_status_data(motor_status);
}

void StarkNode::publish_motor_status() {
    auto motor_status = modbus_get_motor_status(handle_l, slave_id_l);
    auto motor_status_r = modbus_get_motor_status(handle_r, slave_id_r);
    if (!motor_status) {
        RCLCPP_WARN(this->get_logger(), "Failed to get LEFT motor status");
        return;
    }
    if (!motor_status_r) {
        RCLCPP_WARN(this->get_logger(), "Failed to get RIGHT motor status");
        return;
    }

    auto msg = ros2_stark_interfaces::msg::MotorStatus();
    // 使用 std::copy 填充 std::array
    std::copy(motor_status->positions, motor_status->positions + 6, msg.positions.begin());
    std::copy(motor_status->speeds, motor_status->speeds + 6, msg.speeds.begin());
    std::copy(motor_status->currents, motor_status->currents + 6, msg.currents.begin());
    std::copy(motor_status->states, motor_status->states + 6, msg.states.begin());

    motor_status_pub_->publish(msg);
    free_motor_status_data(motor_status);
}

void StarkNode::publish_touch_status() {
    auto touch_status = modbus_get_touch_status(handle_l, slave_id_l);
    auto touch_status_r = modbus_get_touch_status(handle_r, slave_id_r);
    if (!touch_status) {
        RCLCPP_WARN(this->get_logger(), "Failed to get LEFT touch status");
        return;
    }
    if (!touch_status_r) {
        RCLCPP_WARN(this->get_logger(), "Failed to get RIGHT touch status");
        return;
    }

    auto msg = ros2_stark_interfaces::msg::TouchStatus();
    // 填充固定 5 个元素的数组, 代表 5个手指上对应的传感器信息
    for (int i = 0; i < 5; i++) {
        msg.data[i].normal_force1 = touch_status->data[i].normal_force1;
        msg.data[i].normal_force2 = touch_status->data[i].normal_force2;
        msg.data[i].normal_force3 = touch_status->data[i].normal_force3;
        msg.data[i].tangential_force1 = touch_status->data[i].tangential_force1;
        msg.data[i].tangential_force2 = touch_status->data[i].tangential_force2;
        msg.data[i].tangential_force3 = touch_status->data[i].tangential_force3;
        msg.data[i].tangential_direction1 = touch_status->data[i].tangential_direction1;
        msg.data[i].tangential_direction2 = touch_status->data[i].tangential_direction2;
        msg.data[i].tangential_direction3 = touch_status->data[i].tangential_direction3;
        msg.data[i].self_proximity1 = touch_status->data[i].self_proximity1;
        msg.data[i].self_proximity2 = touch_status->data[i].self_proximity2;
        msg.data[i].mutual_proximity = touch_status->data[i].mutual_proximity;
        msg.data[i].status = touch_status->data[i].status;
    }

    touch_status_pub_->publish(msg);
    free_touch_status_data(touch_status);
}

void StarkNode::publish_device_status() {
    // 电量
    auto voltage_l = modbus_get_voltage(handle_l, slave_id_l);
    auto voltage_msg = std_msgs::msg::UInt16();
    voltage_msg.data = voltage_l;
    voltage_pub_->publish(voltage_msg);

}



void StarkNode::joint_cmd_callback_left(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received joint commands left");
    if (msg->position.size() >= 6) {
        uint16_t positions[6];
        for (int i = 0; i < 6; i++) {
            positions[i] = static_cast<uint16_t>(msg->position[i] * 100);  // Scale to 0-100
        }
        RCLCPP_INFO(this->get_logger(), "Left Joint positions: %d, %d, %d, %d, %d, %d",
                    positions[0], positions[1], positions[2], positions[3], positions[4], positions[5]);
        modbus_set_finger_positions(handle_l, slave_id_=0x7e, positions, 6);
    }
}

void StarkNode::joint_cmd_callback_right(const sensor_msgs::msg::JointState::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received joint commands right");
    if (msg->position.size() >= 6) {
        uint16_t positions[6];
        for (int i = 0; i < 6; i++) {
            positions[i] = static_cast<uint16_t>(msg->position[i] * 100);  // Scale to 0-100
        }
        RCLCPP_INFO(this->get_logger(), "Right Joint positions: %d, %d, %d, %d, %d, %d",
                    positions[0], positions[1], positions[2], positions[3], positions[4], positions[5]);
        modbus_set_finger_positions(handle_r, slave_id_=0x7f, positions, 6);
    }
}


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StarkNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}