/**
 * @file omnihand_ctrl.h
 * @author AgiBot-lishuang
 * @date 2025-03
 * @brief Omnihand Controller interface
 */

#ifndef OMNIHAND_CTRL_H
#define OMNIHAND_CTRL_H

#include <cassert>
#include <iostream>
#include <vector>

/**
 * @brief Strongly-typed enumeration for active joints of Omnihand.
 *
 * Each enumerator represents a DOF that can be actively controlled
 * (i.e., it has an actuator).
 */
enum OmnihandActiveJoint {
  ActiveJointhumbRoll = 0,
  ActiveJointThumbAbAd,
  ActiveJointThumbMCP,
  ActiveJointIndexAbAd,
  ActiveJointIndexPIP,
  ActiveJointMiddlePIP,
  ActiveJointRingAbAd,
  ActiveJointRingPIP,
  ActiveJointPinkyAbAd,
  ActiveJointPinkyPIP,

  ActiveJointCount  ///< Helper enumerator to track total number of active joints
};

/**
 * @brief Strongly-typed enumeration for all joints of Omnihand.
 *
 * This includes both actively driven joints and passive joints.
 */
enum OmnihandJoint {
  JointThumbRoll = 0,
  JointThumbAbAd,
  JointThumbMCP,
  JointThumbPIP,
  JointThumbDIP,
  JointIndexAbAd,
  JointIndexPIP,
  JointIndexDIP,
  JointMiddlePIP,
  JointMiddleDIP,
  JointRingAbAd,
  JointRingPIP,
  JointRingDIP,
  JointPinkyAbAd,
  JointPinkyPIP,
  JointPinkyDIP,

  JointCount  ///< Helper enumerator to track total number of joints
};

enum OmnihandGesture {
  GesturePAPER = 0,
  GestureFIST1,
  GestureFIST2,
  GestureOK,
  GestureOneHandedFingerHeart,
  GestureLIKE,
  GestureILY,
  GestureNUM1,
  GestureNUM2,
  GestureNUM3,
  GestureNUM4,
  GestureNUM6,
  GestureNUM8,
  GestureHandHeart1,
  GestureHandHeart2,
  GestureHandHeart3,
  GestureClasping,
};

/**
 * @class OmnihandCtrl
 * @brief A controller class for Omnihand with basic functionalities
 *        like setting gestures, converting positions to actuator inputs, etc.
 */
class OmnihandCtrl {
 private:
  bool hand_type_;  ///< True for left-hand, false for right-hand
  std::vector<int> actuator_max_;
  std::vector<int> actuator_min_;
  int max_iput_ = 4096;
  int min_iput_ = 0;
  // Under is the coef of right hand
  std::vector<double> active_joint_max_ = {1.12, 0.05, 0.8416, 0, 1.48,
                                           1.48, 0.17, 1.48, 0.19, 1.48};
  std::vector<double> motor_max_ = {1.12, 0.05, 1.33, 0, 1.43,
                                    1.43, 0.17, 1.43, 0.19, 1.43};
  std::vector<double> active_joint_min_ = {-0.03, -1.64, 0.0, -0.16, 0.0,
                                           0.0, 0.0, 0.0, 0.0, 0.0};
  std::vector<double> motor_min_ = {-0.03, -1.64, 0.0, -0.16, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0};
  // Up is the coef of right hand
  std::vector<int> left_pos_direction_ = {-1, -1, -1, -1, 1, 1, -1, 1, -1, 1};  // convertion between right hand and left hand
  std::vector<double> finger_pip2dip_poly_ = {0.0, 2.192, -1.425, 0.747,
                                              -0.167};
  std::vector<double> finger_mcp2motor_poly_ = {
      0.00944480234881967, 0.455882677008572, 0.683758090072141,
      -0.916673507519311, 0.459387725400186};
  std::vector<double> finger_motor2mcp_poly_ = {
      -0.000257594494466942, 1.57144033291557, 0.217395210463076,
      -0.768328304426314, 0.248168989312469};

  // y=p1+p2*x+p3*x*x
  std::vector<double> thumb_mcp2pip_poly_ = {0.0, 1.33};
  std::vector<double> thumb_mcp2dip_poly_ = {0.0, 1.846, -0.853, 0.280};  // right hand
  std::vector<double> thumb_pip2mcp_poly_ = {0.0, 1 / 1.33};
  std::vector<double> right_thumb_mcp2motor_poly_ = {
      0.00126371020922368, 0.919140692758276, 0.550958572722048,
      -0.785384985903032, 1.25635285116862};
  std::vector<double> left_thumb_mcp2motor_poly_ = {
      -0.00126371020922368, 0.919140692758276, -0.550958572722048,
      -0.785384985903032, -1.25635285116862};

  std::vector<double> right_thumb_motor2mcp_poly_ = {
      -0.000677604838762652, 1.05175893483608, -0.280133575638901,
      -0.115384415912668, 0.0676128925382166};
  std::vector<double> left_thumb_motor2mcp_poly_ = {
      0.000677604838762652, 1.05175893483608, 0.280133575638901,
      -0.115384415912668, -0.0676128925382166};

  double CalculatePower(const double x, const std::vector<double> &v);
  void ClampJointPos(std::vector<double> &active_joint_pos);
  bool CheckJointPos(const std::vector<double> &active_joint_pos);

  bool CheckActuatorInput(const std::vector<int> &actuator_input);

 public:
  /**
   * @brief Constructor
   * @param hand_type Set true if left-hand, false if right-hand
   */
  OmnihandCtrl(const bool &hand_type);

  /**
   * @brief Destructor
   */
  ~OmnihandCtrl();

  /**
   * @brief Set a predefined hand gesture
   * @param gesture_num Gesture index
   * @return Vector of actuator commands (int)
   */
  std::vector<int> SetHandGesture(const int &gesture_num);

  /**
   * @brief Convert active joint positions to actuator inputs
   * @param active_joint_pos Vector of active joint positions
   * @return Vector of actuator commands (int)
   */
  std::vector<int>
  ActiveJointPos2ActuatorInput(const std::vector<double> &active_joint_pos);

  /**
   * @brief Convert actuator inputs back to active joint positions
   * @param actuator_input Vector of actuator commands
   * @return A vector of active joint positions
   */
  std::vector<double>
  ActuatorInput2ActiveJointPos(const std::vector<int> &actuator_input);

  /**
   * @brief Get all joint positions (active + passive)
   * @param active_joint_pos Current active joints
   * @return A vector representing all Omnihand joints
   */
  std::vector<double>
  GetAllJointPos(const std::vector<double> &active_joint_pos);
};

#endif  // OMNIHAND_CTRL_H
