/*
 * @Author: huangshiheng@agibot.com
 * @Date: 2025-11-06 17:29:45
 * @Description: OmniHand Pro Node implementation
 * 
 * Copyright (c) 2025 by huangshiheng@agibot.com, All Rights Reserved. 
 */

#include "hand_node.h"

namespace omnihand {

OmniHandProNode::OmniHandProNode(uint8_t device_id, uint8_t canfd_id, EHandType hand_type) : Node("omnihand_node" + std::to_string(device_id)) {
  // agibot_hand_ = std::make_shared<AgibotHandO12>(device_id, canfd_id, hand_type);
  agibot_hand_ = AgibotHandO10::createHand(device_id, canfd_id, hand_type);

  if (!agibot_hand_->Init()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to initialize AgibotHandO10 device with ID %d", device_id);
    return;
  }

  bool is_left = (hand_type == EHandType::eLeft);
  std::string topic_prefix = "";
  if (is_left) {
    topic_prefix = "/agihand/omnihand/left/";
  } else {
    topic_prefix = "/agihand/omnihand/right/";
  }

  // Initialize Publishers
  control_mode_publisher_ = this->create_publisher<omnihand_node_msgs::msg::ControlMode>(topic_prefix + "control_mode", 10);
  current_report_publisher_ = this->create_publisher<omnihand_node_msgs::msg::CurrentReport>(topic_prefix + "current_report", 10);
  current_threshold_publisher_ = this->create_publisher<omnihand_node_msgs::msg::CurrentThreshold>(topic_prefix + "current_threshold", 10);
  motor_error_report_publisher_ = this->create_publisher<omnihand_node_msgs::msg::MotorErrorReport>(topic_prefix + "motor_error_report", 10);
  motor_pos_publisher_ = this->create_publisher<omnihand_node_msgs::msg::MotorPos>(topic_prefix + "motor_pos", 10);
  motor_vel_publisher_ = this->create_publisher<omnihand_node_msgs::msg::MotorVel>(topic_prefix + "motor_vel", 10);
  tactile_sensor_publisher_ = this->create_publisher<omnihand_node_msgs::msg::TactileSensor>(topic_prefix + "tactile_sensor", 10);
  temperature_report_publisher_ = this->create_publisher<omnihand_node_msgs::msg::TemperatureReport>(topic_prefix + "temperature_report", 10);

  // Initialize Subscribers
  control_mode_subscriber_ = this->create_subscription<omnihand_node_msgs::msg::ControlMode>(
    topic_prefix + "control_mode_cmd", 1, std::bind(&OmniHandProNode::control_mode_callback, this, std::placeholders::_1));
  current_threshold_subscriber_ = this->create_subscription<omnihand_node_msgs::msg::CurrentThreshold>(
    topic_prefix + "current_threshold_cmd", 1, std::bind(&OmniHandProNode::current_threshold_callback, this, std::placeholders::_1));
  motor_pos_subscriber_ = this->create_subscription<omnihand_node_msgs::msg::MotorPos>(
    topic_prefix + "motor_pos_cmd", 100, std::bind(&OmniHandProNode::motor_pos_callback, this, std::placeholders::_1));
  motor_vel_subscriber_ = this->create_subscription<omnihand_node_msgs::msg::MotorVel>(
    topic_prefix + "motor_vel_cmd", 100, std::bind(&OmniHandProNode::motor_vel_callback, this, std::placeholders::_1));
  motor_angle_subscriber_ = this->create_subscription<omnihand_node_msgs::msg::MotorAngle>(
    topic_prefix + "motor_angle_cmd", 100, std::bind(&OmniHandProNode::motor_angle_callback, this, std::placeholders::_1));

  timer_1hz_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&OmniHandProNode::timer_1hz_callback, this));

  timer_100hz_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&OmniHandProNode::timer_100hz_callback, this));

  RCLCPP_INFO(this->get_logger(), "OmniHand Pro Node initialized device with ID %d", device_id);
}

OmniHandProNode::~OmniHandProNode() {
  RCLCPP_INFO(this->get_logger(), "OmniHand Pro Node destroyed");
}

// Callback implementations
void OmniHandProNode::control_mode_callback(const omnihand_node_msgs::msg::ControlMode::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received control mode command");

  std::vector<unsigned char> vec_ctrl_mode;
  for (auto mode : msg->modes) {
    vec_ctrl_mode.push_back(static_cast<unsigned char>(mode));
  }

  agibot_hand_->SetAllControlMode(vec_ctrl_mode);
}

void OmniHandProNode::mix_control_callback(const omnihand_node_msgs::msg::MixControl::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received mix control command");

  std::vector<MixCtrl> vec_mix_ctrl;
  for (auto mix_control : msg->mix_controls) {
    MixCtrl mix_ctrl;
    mix_ctrl.joint_index_ = static_cast<unsigned char>(mix_control & 0x1F);
    mix_ctrl.ctrl_mode_   = static_cast<unsigned char>((mix_control >> 5) & 0x07);

    mix_ctrl.tgt_posi_   = static_cast<short>((mix_control >> 8)  & 0xFFFF);
    mix_ctrl.tgt_velo_   = static_cast<short>((mix_control >> 24) & 0xFFFF);
    mix_ctrl.tgt_torque_ = static_cast<short>((mix_control >> 40) & 0xFFFF);

    vec_mix_ctrl.push_back(mix_ctrl);
  }
  agibot_hand_->MixCtrlJointMotor(vec_mix_ctrl);
}

void OmniHandProNode::current_threshold_callback(const omnihand_node_msgs::msg::CurrentThreshold::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received current threshold command");

  std::vector<int16_t> vec_current_threshold;
  for (auto threshold : msg->current_thresholds) {
    vec_current_threshold.push_back(static_cast<int16_t>(threshold));
  }
  agibot_hand_->SetAllCurrentThreshold(vec_current_threshold);
}

void OmniHandProNode::motor_pos_callback(const omnihand_node_msgs::msg::MotorPos::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received motor position command with %zu positions", msg->pos.size());

  std::vector<int16_t> vec_pos;
  for (auto pos : msg->pos) {
    vec_pos.push_back(static_cast<int16_t>(pos));
  }
  agibot_hand_->SetAllJointMotorPosi(vec_pos);
}

void OmniHandProNode::motor_vel_callback(const omnihand_node_msgs::msg::MotorVel::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received motor velocity command");

  std::vector<int16_t> vec_velo;
  for (auto vel : msg->vels) {
    vec_velo.push_back(static_cast<int16_t>(vel));
  }
  agibot_hand_->SetAllJointMotorVelo(vec_velo);
}

void OmniHandProNode::motor_angle_callback(const omnihand_node_msgs::msg::MotorAngle::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received motor angle command");

  std::vector<double> vec_angle;
  for (auto angle : msg->angles) {
    vec_angle.push_back(static_cast<double>(angle));
  }
  agibot_hand_->SetAllActiveJointAngles(vec_angle);
}


// Timer callback implementations
void OmniHandProNode::timer_1hz_callback() {
  publish_control_mode();
  publish_current_report();
  publish_current_threshold();
  publish_motor_error_report();
  publish_temperature_report();
}

void OmniHandProNode::timer_100hz_callback() {
  publish_motor_pos();
  publish_motor_vel();
  publish_tactile_sensor();
  publish_motor_angle();
}

// Publisher implementations
void OmniHandProNode::publish_control_mode() {
  auto msg = omnihand_node_msgs::msg::ControlMode();
  msg.header.stamp = this->now();
  msg.header.frame_id = "control_frame";
  auto modes = agibot_hand_->GetAllControlMode();
  msg.modes.resize(modes.size());
  std::transform(modes.begin(), modes.end(), msg.modes.begin(),
                 [](uint8_t mode) { return static_cast<unsigned char>(mode); });

  control_mode_publisher_->publish(msg);
}

void OmniHandProNode::publish_current_report() {
  auto msg = omnihand_node_msgs::msg::CurrentReport();
  msg.header.stamp = this->now();
  msg.header.frame_id = "current_frame";
  auto reports = agibot_hand_->GetAllCurrentReport();
  msg.current_reports = reports;
  current_report_publisher_->publish(msg);
}

void OmniHandProNode::publish_current_threshold() {
  auto msg = omnihand_node_msgs::msg::CurrentThreshold();
  msg.header.stamp = this->now();
  msg.header.frame_id = "current_threshold_frame";
  auto thresholds = agibot_hand_->GetAllCurrentThreshold();
  msg.current_thresholds = thresholds;
  current_threshold_publisher_->publish(msg);
}

void OmniHandProNode::publish_motor_error_report() {
  auto msg = omnihand_node_msgs::msg::MotorErrorReport();
  msg.header.stamp = this->now();
  msg.header.frame_id = "motor_error_frame";

  auto error_reports = agibot_hand_->GetAllErrorReport();
  for (const auto& error_report : error_reports) {
    uint16_t error_code = 0;
    error_code |= (error_report.stalled_ << 0);
    error_code |= (error_report.overheat_ << 1);
    error_code |= (error_report.over_current_ << 2);
    error_code |= (error_report.motor_except_ << 3);
    error_code |= (error_report.commu_except_ << 4);
    error_code |= (error_report.res2_ << 8);
  
    msg.error_reports.push_back(error_code);
  }
  motor_error_report_publisher_->publish(msg);
}

void OmniHandProNode::publish_motor_pos() {
  auto msg = omnihand_node_msgs::msg::MotorPos();
  msg.header.stamp = this->now();
  msg.header.frame_id = "motor_frame";
  auto positions = agibot_hand_->GetAllJointMotorPosi();
  msg.pos = positions;
  motor_pos_publisher_->publish(msg);
}

void OmniHandProNode::publish_motor_vel() {
  auto msg = omnihand_node_msgs::msg::MotorVel();
  msg.header.stamp = this->now();
  msg.header.frame_id = "vel_frame";

  auto velocities = agibot_hand_->GetAllJointMotorVelo();

  msg.vels = velocities;
  motor_vel_publisher_->publish(msg);
}

void OmniHandProNode::publish_tactile_sensor() {
  auto msg = omnihand_node_msgs::msg::TactileSensor();
  msg.header.stamp = this->now();
  msg.header.frame_id = "tactile_frame";
  
  for (int i = 1; i <= 5; i++) {
    auto tactile_sensors = agibot_hand_->GetTactileSensorData(static_cast<EFinger>(i));
    auto msg_data = omnihand_node_msgs::msg::TactileSensorData();

    for (const auto& tactile_sensor : tactile_sensors) {
      msg_data.tactiles.push_back(i);
    }

    msg.tactile_datas.push_back(msg_data);
  }

  tactile_sensor_publisher_->publish(msg);
}

void OmniHandProNode::publish_temperature_report() {
  auto msg = omnihand_node_msgs::msg::TemperatureReport();
  msg.header.stamp = this->now();
  msg.header.frame_id = "temperature_frame";

  auto temperatures = agibot_hand_->GetAllTemperatureReport();
  msg.temperature_reports = temperatures;
  temperature_report_publisher_->publish(msg);
}

void OmniHandProNode::publish_motor_angle() {
  auto msg = omnihand_node_msgs::msg::MotorAngle();
  msg.header.stamp = this->now();
  msg.header.frame_id = "angle_frame";

  auto angles = agibot_hand_->GetAllActiveJointAngles();
  msg.angles = angles;
}

} // namespace omnihand