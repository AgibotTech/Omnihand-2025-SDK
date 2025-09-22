// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand 2025 SDK is licensed under Mulan PSL v2.

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "c_agibot_hand_base.h"

namespace py = pybind11;

PYBIND11_MODULE(agibot_hand_core, m) {
  m.doc() = "AgibotHand Python Interface";

  // Bind CommuParams structure
  py::class_<CommuParams>(m, "CommuParams")
      .def(py::init<>())
      .def_readwrite("bitrate_", &CommuParams::bitrate_)
      .def_readwrite("sample_point_", &CommuParams::sample_point_)
      .def_readwrite("dbitrate_", &CommuParams::dbitrate_)
      .def_readwrite("dsample_point_", &CommuParams::dsample_point_);

  // Bind DeviceInfo structure
  py::class_<DeviceInfo>(m, "DeviceInfo")
      .def(py::init<>())
      .def_readwrite("device_id", &DeviceInfo::deviceId)
      .def_readwrite("commu_params", &DeviceInfo::commuParams)
      .def("__str__", &DeviceInfo::toString);

  // Bind Version structure
  py::class_<Version>(m, "Version")
      .def(py::init<>())
      .def_readwrite("major_", &Version::major_)
      .def_readwrite("minor_", &Version::minor_)
      .def_readwrite("patch_", &Version::patch_)
      .def_readwrite("res_", &Version::res_);

  // Bind VendorInfo structure
  py::class_<VendorInfo>(m, "VendorInfo")
      .def(py::init<>())
      .def_readwrite("product_model", &VendorInfo::productModel)
      .def_readwrite("product_seq_num", &VendorInfo::productSeqNum)
      .def_readwrite("hardware_version", &VendorInfo::hardwareVersion)
      .def_readwrite("software_version", &VendorInfo::softwareVersion)
      .def_readwrite("voltage", &VendorInfo::voltage)
      .def_readwrite("dof", &VendorInfo::dof)
      .def("__str__", &VendorInfo::toString);

  // Bind JointMotorErrorReport structure with bit field accessors
  py::class_<JointMotorErrorReport>(m, "JointMotorErrorReport")
      .def(py::init<>())
      .def_property(
          "stalled",
          [](const JointMotorErrorReport &self) { return static_cast<bool>(self.stalled_); },
          [](JointMotorErrorReport &self, bool value) { self.stalled_ = static_cast<unsigned char>(value); })
      .def_property(
          "overheat",
          [](const JointMotorErrorReport &self) { return static_cast<bool>(self.overheat_); },
          [](JointMotorErrorReport &self, bool value) { self.overheat_ = static_cast<unsigned char>(value); })
      .def_property(
          "over_current",
          [](const JointMotorErrorReport &self) { return static_cast<bool>(self.over_current_); },
          [](JointMotorErrorReport &self, bool value) { self.over_current_ = static_cast<unsigned char>(value); })
      .def_property(
          "motor_except",
          [](const JointMotorErrorReport &self) { return static_cast<bool>(self.motor_except_); },
          [](JointMotorErrorReport &self, bool value) { self.motor_except_ = static_cast<unsigned char>(value); })
      .def_property(
          "commu_except",
          [](const JointMotorErrorReport &self) { return static_cast<bool>(self.commu_except_); },
          [](JointMotorErrorReport &self, bool value) { self.commu_except_ = static_cast<unsigned char>(value); });

  // Bind MixCtrl structure
  py::class_<MixCtrl>(m, "MixCtrl")
      .def(py::init<>())
      .def_property(
          "joint_index",
          [](const MixCtrl &self) { return static_cast<int>(self.joint_index_); },
          [](MixCtrl &self, int value) { self.joint_index_ = static_cast<unsigned char>(value); })
      .def_property(
          "ctrl_mode",
          [](const MixCtrl &self) { return static_cast<int>(self.ctrl_mode_); },
          [](MixCtrl &self, int value) { self.ctrl_mode_ = static_cast<unsigned char>(value); })
      .def_readwrite("tgt_posi", &MixCtrl::tgt_posi_)
      .def_readwrite("tgt_velo", &MixCtrl::tgt_velo_)
      .def_readwrite("tgt_torque", &MixCtrl::tgt_torque_);

  // Bind base class
  py::class_<AgibotHandBase>(m, "AgibotHandBase")
      .def_static(
          "create_hand",
          [](bool use_can, unsigned char device_id, int hand_type) {
            return AgibotHandBase::createHand(
                use_can,
                device_id,
                static_cast<EHandType>(hand_type));
          },
          py::arg("use_can") = true,
          py::arg("device_id") = DEFAULT_DEVICE_ID,
          py::arg("hand_type") = 0)
      .def("set_device_id", &AgibotHandBase::SetDeviceId)
      .def("get_vendor_info", &AgibotHandBase::GetVendorInfo)
      .def("get_device_info", &AgibotHandBase::GetDeviceInfo)
      .def("set_joint_position", &AgibotHandBase::SetJointMotorPosi)
      .def("get_joint_position", &AgibotHandBase::GetJointMotorPosi)
      .def("set_all_joint_positions", &AgibotHandBase::SetAllJointMotorPosi)
      .def("get_all_joint_positions", &AgibotHandBase::GetAllJointMotorPosi)
#if !DISABLE_FUNC
      .def("set_active_joint_angle", &AgibotHandBase::SetActiveJointAngle)
      .def("get_active_joint_angle", &AgibotHandBase::GetActiveJointAngle)
#endif
      .def("set_all_active_joint_angles", &AgibotHandBase::SetAllActiveJointAngles)
      .def("get_all_active_joint_angles", &AgibotHandBase::GetAllActiveJointAngles)
      .def("get_all_joint_angles", &AgibotHandBase::GetAllJointAngles)
      .def("set_joint_velocity", &AgibotHandBase::SetJointMotorVelo)
      .def("get_joint_velocity", &AgibotHandBase::GetJointMotorVelo)
      .def("set_all_joint_velocities", &AgibotHandBase::SetAllJointMotorVelo)
      .def("get_all_joint_velocities", &AgibotHandBase::GetAllJointMotorVelo)
#if !DISABLE_FUNC
      .def("set_joint_torque", &AgibotHandBase::SetJointMotorTorque)
      .def("get_joint_torque", &AgibotHandBase::GetJointMotorTorque)
      .def("set_all_joint_torques", &AgibotHandBase::SetAllJointMotorTorque)
      .def("get_all_joint_torques", &AgibotHandBase::GetAllJointMotorTorque)
#endif
      .def("get_touch_sensor_data", [](AgibotHandBase &self, int finger_index) {
        return self.GetTouchSensorData(static_cast<EFinger>(finger_index));
      })
      .def("set_control_mode", [](AgibotHandBase &self, int joint_motor_index, int mode) {
        self.SetControlMode(joint_motor_index, static_cast<EControlMode>(mode));
      })
      .def("get_control_mode", [](AgibotHandBase &self, int joint_motor_index) -> int {
        return static_cast<int>(self.GetControlMode(joint_motor_index));
      })
      .def("set_all_control_modes", &AgibotHandBase::SetAllControlMode)
      .def("get_all_control_modes", &AgibotHandBase::GetAllControlMode)
      .def("set_current_threshold", &AgibotHandBase::SetCurrentThreshold)
      .def("get_current_threshold", &AgibotHandBase::GetCurrentThreshold)
      .def("set_all_current_thresholds", &AgibotHandBase::SetAllCurrentThreshold)
      .def("get_all_current_thresholds", &AgibotHandBase::GetAllCurrentThreshold)
      .def("mix_ctrl_joint_motor", &AgibotHandBase::MixCtrlJointMotor)
      .def("get_error_report", &AgibotHandBase::GetErrorReport)
      .def("get_all_error_reports", &AgibotHandBase::GetAllErrorReport)
      .def("set_error_report_period", &AgibotHandBase::SetErrorReportPeriod)
      .def("set_all_error_report_periods", &AgibotHandBase::SetAllErrorReportPeriod)
      .def("get_temperature_report", &AgibotHandBase::GetTemperatureReport)
      .def("get_all_temperature_reports", &AgibotHandBase::GetAllTemperatureReport)
      .def("set_temperature_report_period", &AgibotHandBase::SetTemperReportPeriod)
      .def("set_all_temperature_report_periods", &AgibotHandBase::SetAllTemperReportPeriod)
      .def("get_current_report", &AgibotHandBase::GetCurrentReport)
      .def("get_all_current_reports", &AgibotHandBase::GetAllCurrentReport)
      .def("set_current_report_period", &AgibotHandBase::SetCurrentReportPeriod)
      .def("set_all_current_report_periods", &AgibotHandBase::SetAllCurrentReportPeriod)
      .def("show_data_details", &AgibotHandBase::ShowDataDetails);
}