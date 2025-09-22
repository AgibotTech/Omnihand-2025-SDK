// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand 2025 SDK is licensed under Mulan PSL v2.

#pragma once

#include "c_agibot_hand_base.h"
#include "proto.h"
#include "rs_485_device/rs_485_device.h"

#include "export_symbols.h"

#define DEFAULT_DEVICE_ID 0x01

#define DISABLE_FUNC 1

constexpr uint8_t HEADER[] = {0xEE, 0xAA, 0x1, 0x0};

class AGIBOT_EXPORT AgibotHandRsO10 : public AgibotHandBase {
 public:
  AgibotHandRsO10();

  ~AgibotHandRsO10() override = default;

  VendorInfo GetVendorInfo() override;

  DeviceInfo GetDeviceInfo() override;

  void SetDeviceId(unsigned char device_id) override;

  void SetJointMotorPosi(unsigned char joint_motor_index, int16_t posi) override;

  int16_t GetJointMotorPosi(unsigned char joint_motor_index) override;

  void SetAllJointMotorPosi(std::vector<int16_t> vec_posi) override;

  std::vector<int16_t> GetAllJointMotorPosi() override;
#if !DISABLE_FUNC

  void SetActiveJointAngle(unsigned char joint_motor_index, double angle) override;

  double GetActiveJointAngle(unsigned char joint_motor_index) override;
#endif

  void SetAllActiveJointAngles(std::vector<double> vec_angle) override;

  std::vector<double> GetAllActiveJointAngles() override;

  std::vector<double> GetAllJointAngles() override;

#if !DISABLE_FUNC

  void SetJointMotorTorque(unsigned char joint_motor_index, int16_t torque) override;

  int16_t GetJointMotorTorque(unsigned char joint_motor_index) override;

  void SetAllJointMotorTorque(std::vector<int16_t> vec_torque) override;

  std::vector<int16_t> GetAllJointMotorTorque() override;
#endif

  void SetJointMotorVelo(unsigned char joint_motor_index, int16_t velo) override;

  int16_t GetJointMotorVelo(unsigned char joint_motor_index) override;

  void SetAllJointMotorVelo(std::vector<int16_t> vec_velo) override;

  std::vector<int16_t> GetAllJointMotorVelo() override;

  std::vector<uint8_t> GetTouchSensorData(EFinger eFinger) override;

  void SetControlMode(unsigned char joint_motor_index, EControlMode mode) override;

  EControlMode GetControlMode(unsigned char joint_motor_index) override;

  void SetAllControlMode(std::vector<unsigned char> vec_ctrl_mode) override;

  std::vector<unsigned char> GetAllControlMode() override;

  void SetCurrentThreshold(unsigned char joint_motor_index, int16_t current_threshold) override;

  int16_t GetCurrentThreshold(unsigned char joint_motor_index) override;

  void SetAllCurrentThreshold(std::vector<int16_t> vec_current_threshold) override;

  std::vector<int16_t> GetAllCurrentThreshold() override;

  void MixCtrlJointMotor(std::vector<MixCtrl> vec_mix_ctrl) override;

  JointMotorErrorReport GetErrorReport(unsigned char joint_motor_index) override;

  std::vector<JointMotorErrorReport> GetAllErrorReport() override;

  void SetErrorReportPeriod(unsigned char joint_motor_index, uint16_t period) override;

  void SetAllErrorReportPeriod(std::vector<uint16_t> vec_period) override;

  uint16_t GetTemperatureReport(unsigned char joint_motor_index) override;

  std::vector<uint16_t> GetAllTemperatureReport() override;

  void SetTemperReportPeriod(unsigned char joint_motor_index, uint16_t period) override;

  void SetAllTemperReportPeriod(std::vector<uint16_t> vec_period) override;

  int16_t GetCurrentReport(unsigned char joint_motor_index) override;

  std::vector<uint16_t> GetAllCurrentReport() override;

  void SetCurrentReportPeriod(unsigned char joint_motor_index, uint16_t period) override;

  void SetAllCurrentReportPeriod(std::vector<uint16_t> vec_period) override;

  void ShowDataDetails(bool show) const override;

 protected:
  std::unique_ptr<UartRs485Interface> handrs485_interface_;
};
