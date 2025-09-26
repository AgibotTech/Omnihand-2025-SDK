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
  py::class_<AgibotHandO10>(m, "AgibotHandO10")
      .def_static(
          "create_hand",
          [](unsigned char device_id, int hand_type, std::string_view cfg_path) {
            return AgibotHandO10::createHand(
                device_id,
                static_cast<EHandType>(hand_type),
                cfg_path);
          },
          py::arg("device_id") = DEFAULT_DEVICE_ID,
          py::arg("hand_type") = 0,
          py::arg("cfg_path") = "")
      .def("set_device_id", &AgibotHandO10::SetDeviceId)
      .def("get_vendor_info", &AgibotHandO10::GetVendorInfo)
      .def("get_device_info", &AgibotHandO10::GetDeviceInfo)
      .def("set_joint_position", &AgibotHandO10::SetJointMotorPosi)
      .def("get_joint_position", &AgibotHandO10::GetJointMotorPosi)
      .def("set_all_joint_positions", &AgibotHandO10::SetAllJointMotorPosi)
      .def("get_all_joint_positions", &AgibotHandO10::GetAllJointMotorPosi)
#if !DISABLE_FUNC
      .def("set_active_joint_angle", &AgibotHandO10::SetActiveJointAngle)
      .def("get_active_joint_angle", &AgibotHandO10::GetActiveJointAngle)
#endif
      .def("set_all_active_joint_angles", &AgibotHandO10::SetAllActiveJointAngles)
      .def("get_all_active_joint_angles", &AgibotHandO10::GetAllActiveJointAngles)
      .def("get_all_joint_angles", &AgibotHandO10::GetAllJointAngles)
      .def("set_joint_velocity", &AgibotHandO10::SetJointMotorVelo)
      .def("get_joint_velocity", &AgibotHandO10::GetJointMotorVelo)
      .def("set_all_joint_velocities", &AgibotHandO10::SetAllJointMotorVelo)
      .def("get_all_joint_velocities", &AgibotHandO10::GetAllJointMotorVelo)
#if !DISABLE_FUNC
      .def("set_joint_torque", &AgibotHandO10::SetJointMotorTorque)
      .def("get_joint_torque", &AgibotHandO10::GetJointMotorTorque)
      .def("set_all_joint_torques", &AgibotHandO10::SetAllJointMotorTorque)
      .def("get_all_joint_torques", &AgibotHandO10::GetAllJointMotorTorque)
#endif
      .def("get_tactile_sensor_data", [](AgibotHandO10 &self, int finger_index) {
        return self.GetTactileSensorData(static_cast<EFinger>(finger_index));
      })
      .def("set_control_mode", [](AgibotHandO10 &self, int joint_motor_index, int mode) {
        self.SetControlMode(joint_motor_index, static_cast<EControlMode>(mode));
      })
      .def("get_control_mode", [](AgibotHandO10 &self, int joint_motor_index) -> int {
        return static_cast<int>(self.GetControlMode(joint_motor_index));
      })
      .def("set_all_control_modes", &AgibotHandO10::SetAllControlMode)
      .def("get_all_control_modes", &AgibotHandO10::GetAllControlMode)
      .def("set_current_threshold", &AgibotHandO10::SetCurrentThreshold)
      .def("get_current_threshold", &AgibotHandO10::GetCurrentThreshold)
      .def("set_all_current_thresholds", &AgibotHandO10::SetAllCurrentThreshold)
      .def("get_all_current_thresholds", &AgibotHandO10::GetAllCurrentThreshold)
      .def("mix_ctrl_joint_motor", &AgibotHandO10::MixCtrlJointMotor)
      .def("get_error_report", &AgibotHandO10::GetErrorReport)
      .def("get_all_error_reports", &AgibotHandO10::GetAllErrorReport)
#if !DISABLE_FUNC
      .def("set_error_report_period", &AgibotHandO10::SetErrorReportPeriod)
      .def("set_all_error_report_periods", &AgibotHandO10::SetAllErrorReportPeriod)
#endif
      .def("get_temperature_report", &AgibotHandO10::GetTemperatureReport)
      .def("get_all_temperature_reports", &AgibotHandO10::GetAllTemperatureReport)
#if !DISABLE_FUNC
      .def("set_temperature_report_period", &AgibotHandO10::SetTemperReportPeriod)
      .def("set_all_temperature_report_periods", &AgibotHandO10::SetAllTemperReportPeriod)
#endif
      .def("get_current_report", &AgibotHandO10::GetCurrentReport)
      .def("get_all_current_reports", &AgibotHandO10::GetAllCurrentReport)
#if !DISABLE_FUNC
      .def("set_current_report_period", &AgibotHandO10::SetCurrentReportPeriod)
      .def("set_all_current_report_periods", &AgibotHandO10::SetAllCurrentReportPeriod)
#endif
      .def("show_data_details", &AgibotHandO10::ShowDataDetails);
}