#include "kinematics_solver.h"

OmnihandCtrl::OmnihandCtrl(const bool &hand_type) : hand_type_(hand_type) {
  auto left_active_joint_max = active_joint_max_;
  auto left_active_joint_min = active_joint_min_;
  auto left_motor_max = motor_max_;
  auto left_motor_min = motor_min_;

  for (size_t i = 0; i < ActiveJointCount; i++) {
    if (left_pos_direction_[i] == -1) {
      left_active_joint_max[i] = -active_joint_min_[i];
      left_active_joint_min[i] = -active_joint_max_[i];
      left_motor_max[i] = -motor_min_[i];
      left_motor_min[i] = -motor_max_[i];
    }
  }
  if (hand_type_) {
    active_joint_min_ = left_active_joint_min;
    active_joint_max_ = left_active_joint_max;
    motor_max_ = left_motor_max;
    motor_min_ = left_motor_min;
    thumb_mcp2dip_poly_[2] = -thumb_mcp2dip_poly_[2];
    actuator_max_ = {4096, 0, 4096, 0, 0, 0, 4096, 0, 0, 0};
    actuator_min_ = {0, 4096, 0, 4096, 4096, 4096, 0, 4096, 4096, 4096};
  } else {
    actuator_max_ = {0, 0, 0, 0, 0, 0, 4096, 0, 0, 0};
    actuator_min_ = {4096, 4096, 4096, 4096, 4096, 4096, 0, 4096, 4096, 4096};
  }
}

OmnihandCtrl::~OmnihandCtrl() {}

std::vector<int> OmnihandCtrl::SetHandGesture(const int &gesture_num) {
  std::vector<double> active_joint_pos(ActiveJointCount, 0.0);
  std::vector<int> actuator_input;
  std::vector<int> pos_direction(ActiveJointCount, 1);
  if (hand_type_) {
    pos_direction = left_pos_direction_;
  }

  switch (gesture_num) {
    case GesturePAPER:
      active_joint_pos = {0.58, -0.21, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      break;

    case GestureFIST1:
      active_joint_pos = {0.43, -0.3, 0.66, 0.0, 1.48,
                          1.48, 0.0, 1.48, 0.0, 1.48};
      break;

    case GestureFIST2:
      active_joint_pos = {0.5, -1.0, 0.75, 0.0, 1.48, 1.48, 0.0, 1.48, 0.0, 1.48};
      break;

    case GestureOK:
      active_joint_pos = {-0.03, -1.51, 0.7, -0.16, 0.85,
                          0.21, 0.07, 0.153, 0.107, 0.1};
      break;

    case GestureOneHandedFingerHeart:
      active_joint_pos = {0.8, -0.4, 0.47, 0.0, 0.82, 1.48, 0.0, 1.48, 0.0, 1.48};
      break;

    case GestureLIKE:
      active_joint_pos = {0.27, 0.0, 0.0, 0.0, 1.48, 1.48, 0.0, 1.48, 0.0, 1.48};
      break;

    case GestureILY:
      active_joint_pos = {0.33, 0.0, 0.0, -0.1, 0.0, 1.48, 0.07, 1.48, 0.11, 0.0};
      break;

    case GestureNUM1:
      active_joint_pos = {0.32, -1.12, 0.79, -0.06, 0.0,
                          1.48, 0.0, 1.48, 0.0, 1.48};
      break;

    case GestureNUM2:
      active_joint_pos = {0.48, -1.5, 0.79, -0.16, 0.0,
                          0.0, 0.0, 1.48, 0.0, 1.48};
      break;

    case GestureNUM3:
      active_joint_pos = {0.64, -1.48, 0.81, -0.16, 0.0,
                          0.0, 0.09, 0.0, 0.09, 1.48};
      break;

    case GestureNUM4:
      active_joint_pos = {0.64, -1.48, 0.81, -0.16, 0.0,
                          0.0, 0.07, 0.0, 0.15, 0.0};
      break;

    case GestureNUM6:
      active_joint_pos = {0.40, 0.0, 0.0, 0.0, 1.48, 1.48, 0.05, 1.48, 0.17, 0.0};
      break;

    case GestureNUM8:
      active_joint_pos = {0.40, 0.0, 0.0, 0.0, 0.0, 1.48, 0.0, 1.48, 0.0, 1.48};
      break;

    case GestureHandHeart1:
      active_joint_pos = {-0.03, -1.36, 0.0, 0.0, 0.65,
                          0.65, 0.0, 0.65, 0.0, 0.65};
      break;

    case GestureHandHeart2:
      active_joint_pos = {0.30, -0.1, 0.66, 0.0, 1.1, 1.1, 0.0, 1.1, 0.0, 1.1};
      break;

    case GestureHandHeart3:
      active_joint_pos = {0.0, -1.56, 0.46, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      break;

    case GestureClasping:
      active_joint_pos = {0.5, -0.8, 0.2, -0.16, 0.6, 0.6, 0.17, 0.6, 0.17, 0.6};
      break;

    default:
      break;
  }
  for (size_t i = 0; i < ActiveJointCount; i++) {
    active_joint_pos[i] = active_joint_pos[i] * pos_direction[i];
  }
  ClampJointPos(active_joint_pos);
  actuator_input = ActiveJointPos2ActuatorInput(active_joint_pos);
  return actuator_input;
}

std::vector<int> OmnihandCtrl::ActiveJointPos2ActuatorInput(
    const std::vector<double> &active_joint_pos) {
  // check active_joint_pos size

  assert(active_joint_pos.size() == ActiveJointCount);
  CheckJointPos(active_joint_pos);
  std::vector<double> hand_pos = active_joint_pos;
  if (!hand_type_) {
    hand_pos[ActiveJointThumbMCP] = CalculatePower(
        hand_pos[ActiveJointThumbMCP], right_thumb_mcp2motor_poly_);
  } else {
    hand_pos[ActiveJointThumbMCP] = CalculatePower(
        hand_pos[ActiveJointThumbMCP], left_thumb_mcp2motor_poly_);
  }
  hand_pos[ActiveJointIndexPIP] =
      CalculatePower(hand_pos[ActiveJointIndexPIP], finger_mcp2motor_poly_);
  hand_pos[ActiveJointMiddlePIP] =
      CalculatePower(hand_pos[ActiveJointMiddlePIP], finger_mcp2motor_poly_);
  hand_pos[ActiveJointRingPIP] =
      CalculatePower(hand_pos[ActiveJointRingPIP], finger_mcp2motor_poly_);
  hand_pos[ActiveJointPinkyPIP] =
      CalculatePower(hand_pos[ActiveJointPinkyPIP], finger_mcp2motor_poly_);

  std::vector<int> actuator_input;
  for (size_t i = 0; i < ActiveJointCount; i++) {
    // std::cout<<"motor: "<<hand_pos[i]<<" "<<motor_min_[i]<<" "<<actuator_min_[i]<<std::endl;
    actuator_input.push_back(int((hand_pos[i] - motor_min_[i]) *
                                 (actuator_max_[i] - actuator_min_[i]) /
                                 (motor_max_[i] - motor_min_[i])) +
                             actuator_min_[i]);
  }
  return actuator_input;
}

std::vector<double> OmnihandCtrl::ActuatorInput2ActiveJointPos(
    const std::vector<int> &actuator_input) {
  // check actuator_input size
  assert(actuator_input.size() == ActiveJointCount);
  CheckActuatorInput(actuator_input);

  std::vector<int> hand_input = actuator_input;

  std::vector<double> active_joint_pos;
  for (size_t i = 0; i < ActiveJointCount; i++) {
    active_joint_pos.push_back((hand_input[i] - actuator_min_[i]) *
                                   (motor_max_[i] - motor_min_[i]) /
                                   (actuator_max_[i] - actuator_min_[i]) +
                               motor_min_[i]);
  }
  if (!hand_type_) {
    active_joint_pos[ActiveJointThumbMCP] = CalculatePower(
        active_joint_pos[ActiveJointThumbMCP], right_thumb_motor2mcp_poly_);
  } else {
    active_joint_pos[ActiveJointThumbMCP] = CalculatePower(
        active_joint_pos[ActiveJointThumbMCP], left_thumb_motor2mcp_poly_);
  }
  active_joint_pos[ActiveJointIndexPIP] = CalculatePower(
      active_joint_pos[ActiveJointIndexPIP], finger_motor2mcp_poly_);
  active_joint_pos[ActiveJointMiddlePIP] = CalculatePower(
      active_joint_pos[ActiveJointMiddlePIP], finger_motor2mcp_poly_);
  active_joint_pos[ActiveJointRingPIP] = CalculatePower(
      active_joint_pos[ActiveJointRingPIP], finger_motor2mcp_poly_);
  active_joint_pos[ActiveJointPinkyPIP] = CalculatePower(
      active_joint_pos[ActiveJointPinkyPIP], finger_motor2mcp_poly_);
  ClampJointPos(active_joint_pos);
  return active_joint_pos;
}

std::vector<double>
OmnihandCtrl::GetAllJointPos(const std::vector<double> &active_joint_pos) {
  assert(active_joint_pos.size() == ActiveJointCount);
  std::vector<double> all_joint_pos(JointCount, 0);
  all_joint_pos[JointThumbRoll] = active_joint_pos[ActiveJointhumbRoll];
  all_joint_pos[JointThumbAbAd] = active_joint_pos[ActiveJointThumbAbAd];
  all_joint_pos[JointThumbMCP] = active_joint_pos[ActiveJointThumbMCP];
  all_joint_pos[JointThumbPIP] = CalculatePower(
      active_joint_pos[ActiveJointThumbMCP], thumb_mcp2pip_poly_);
  all_joint_pos[JointThumbDIP] = CalculatePower(
      active_joint_pos[ActiveJointThumbMCP], thumb_mcp2dip_poly_);
  all_joint_pos[JointIndexAbAd] = active_joint_pos[ActiveJointIndexAbAd];
  all_joint_pos[JointIndexPIP] = active_joint_pos[ActiveJointIndexPIP];
  all_joint_pos[JointIndexDIP] = CalculatePower(
      active_joint_pos[ActiveJointIndexPIP], finger_pip2dip_poly_);
  all_joint_pos[JointMiddlePIP] = active_joint_pos[ActiveJointMiddlePIP];
  all_joint_pos[JointMiddleDIP] = CalculatePower(
      active_joint_pos[ActiveJointMiddlePIP], finger_pip2dip_poly_);
  all_joint_pos[JointRingAbAd] = active_joint_pos[ActiveJointRingAbAd];
  all_joint_pos[JointRingPIP] = active_joint_pos[ActiveJointRingPIP];
  all_joint_pos[JointRingDIP] = CalculatePower(
      active_joint_pos[ActiveJointRingPIP], finger_pip2dip_poly_);
  all_joint_pos[JointPinkyAbAd] = active_joint_pos[ActiveJointPinkyAbAd];
  all_joint_pos[JointPinkyPIP] = active_joint_pos[ActiveJointPinkyPIP];
  all_joint_pos[JointPinkyDIP] = CalculatePower(
      active_joint_pos[ActiveJointPinkyPIP], finger_pip2dip_poly_);
  return all_joint_pos;
}

bool OmnihandCtrl::CheckJointPos(const std::vector<double> &active_joint_pos) {
  assert(active_joint_pos.size() == ActiveJointCount);
  bool is_safe = true;
  for (size_t i = 0; i < ActiveJointCount; i++) {
    if (active_joint_pos[i] >= active_joint_min_[i] &&
        active_joint_pos[i] <= active_joint_max_[i]) {
    } else {
      is_safe = false;
    }
  }
  return is_safe;
}

bool OmnihandCtrl::CheckActuatorInput(const std::vector<int> &actuator_input) {
  assert(actuator_input.size() == ActiveJointCount);
  bool is_safe = true;
  for (size_t i = 0; i < ActiveJointCount; i++) {
    if (actuator_input[i] >= max_iput_ && actuator_input[i] <= min_iput_) {
    } else {
      is_safe = false;
    }
  }
  return is_safe;
}

double OmnihandCtrl::CalculatePower(const double x,
                                    const std::vector<double> &v) {
  double result = 0.0;
  double power = 1.0;

  for (size_t i = 0; i < v.size(); ++i) {
    result += v[i] * power;
    power *= x;
  }
  return result;
}

void OmnihandCtrl::ClampJointPos(std::vector<double> &active_joint_pos) {
  for (size_t i = 0; i < ActiveJointCount; i++) {
    active_joint_pos[i] =
        std::min(std::max(active_joint_min_[i], active_joint_pos[i]),
                 active_joint_max_[i]);
  }
}
