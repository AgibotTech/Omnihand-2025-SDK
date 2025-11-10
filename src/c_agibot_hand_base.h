// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand 2025 SDK is licensed under Mulan PSL v2.

/**
 * @file c_agibot_hand_base.h
 * @brief 灵巧手基类
 * @author WSJ
 * @date 25-8-1
 **/

#pragma once

#include <memory>
#include <string>
#include <vector>
#include "export_symbols.h"
#include "kinematics_solver/kinematics_solver.h"
#include "proto.h"
#include "yaml-cpp/yaml.h"

#define DEFAULT_DEVICE_ID 0x01
#define DISABLE_FUNC 1

/**
 * @brief 灵巧手基类
 */
class AGIBOT_EXPORT AgibotHandO10 {
 public:
  struct HardwareConf {
    std::string device = "can";
    YAML::Node options;
  };

 public:
  /**
   * @brief 工厂方法，创建具体的灵巧手实例
   */
  static std::unique_ptr<AgibotHandO10> createHand(
      unsigned char device_id,
      unsigned char canfd_id,
      EHandType hand_type);
  /**
   * @brief 构造函数
   * @param device_id 设备Id
   * @param hand_type 手型(左手/右手)
   */

  AgibotHandO10() = default;
  virtual ~AgibotHandO10() = default;

  // 基本信息接口
  virtual VendorInfo GetVendorInfo() = 0;
  virtual DeviceInfo GetDeviceInfo() = 0;
  virtual void SetDeviceId(unsigned char device_id) = 0;
  void init(unsigned char device_id, EHandType hand_type) {
    device_id_ = device_id;
    is_left_hand_ = (hand_type == EHandType::eLeft);
    kinematics_solver_ptr_ = std::make_unique<OmnihandCtrl>(is_left_hand_);
  }

  // 关节电机位置控制
  virtual void SetJointMotorPosi(unsigned char joint_motor_index, int16_t posi) = 0;
  virtual int16_t GetJointMotorPosi(unsigned char joint_motor_index) = 0;
  virtual void SetAllJointMotorPosi(std::vector<int16_t> vec_posi) = 0;
  virtual std::vector<int16_t> GetAllJointMotorPosi() = 0;

  // 关节角度控制
  virtual void SetAllActiveJointAngles(std::vector<double> vec_angle) = 0;
  virtual std::vector<double> GetAllActiveJointAngles() = 0;
  virtual std::vector<double> GetAllJointAngles() = 0;

  // 速度控制
  virtual void SetJointMotorVelo(unsigned char joint_motor_index, int16_t velo) = 0;
  virtual int16_t GetJointMotorVelo(unsigned char joint_motor_index) = 0;
  virtual void SetAllJointMotorVelo(std::vector<int16_t> vec_velo) = 0;
  virtual std::vector<int16_t> GetAllJointMotorVelo() = 0;

  // 传感器接口
  virtual std::vector<uint8_t> GetTactileSensorData(EFinger eFinger) = 0;

  // 控制模式
  virtual void SetControlMode(unsigned char joint_motor_index, EControlMode mode) = 0;
  virtual EControlMode GetControlMode(unsigned char joint_motor_index) = 0;
  virtual void SetAllControlMode(std::vector<unsigned char> vec_ctrl_mode) = 0;
  virtual std::vector<unsigned char> GetAllControlMode() = 0;

  // 电流控制
  virtual void SetCurrentThreshold(unsigned char joint_motor_index, int16_t current_threshold) = 0;
  virtual int16_t GetCurrentThreshold(unsigned char joint_motor_index) = 0;
  virtual void SetAllCurrentThreshold(std::vector<int16_t> vec_current_threshold) = 0;
  virtual std::vector<int16_t> GetAllCurrentThreshold() = 0;

  // 混合控制
  virtual void MixCtrlJointMotor(std::vector<MixCtrl> vec_mix_ctrl) = 0;

  // 错误报告
  virtual JointMotorErrorReport GetErrorReport(unsigned char joint_motor_index) = 0;
  virtual std::vector<JointMotorErrorReport> GetAllErrorReport() = 0;
#if !DISABLE_FUNC
  virtual void SetErrorReportPeriod(unsigned char joint_motor_index, uint16_t period) = 0;
  virtual void SetAllErrorReportPeriod(std::vector<uint16_t> vec_period) = 0;
#endif
  // 温度报告
  virtual uint16_t GetTemperatureReport(unsigned char joint_motor_index) = 0;
  virtual std::vector<uint16_t> GetAllTemperatureReport() = 0;
#if !DISABLE_FUNC
  virtual void SetTemperReportPeriod(unsigned char joint_motor_index, uint16_t period) = 0;
  virtual void SetAllTemperReportPeriod(std::vector<uint16_t> vec_period) = 0;
#endif
  // 电流报告
  virtual int16_t GetCurrentReport(unsigned char joint_motor_index) = 0;
  virtual std::vector<uint16_t> GetAllCurrentReport() = 0;
#if !DISABLE_FUNC
  virtual void SetCurrentReportPeriod(unsigned char joint_motor_index, uint16_t period) = 0;
  virtual void SetAllCurrentReportPeriod(std::vector<uint16_t> vec_period) = 0;
#endif
  // 调试接口
  virtual void ShowDataDetails(bool show) const = 0;

 protected:
  // todo OTA

 protected:
  std::unique_ptr<OmnihandCtrl> kinematics_solver_ptr_;
  unsigned char device_id_{DEFAULT_DEVICE_ID};
  bool is_left_hand_{true};
};
