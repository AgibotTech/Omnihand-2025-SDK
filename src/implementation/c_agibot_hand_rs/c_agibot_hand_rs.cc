// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand 2025 SDK is licensed under Mulan PSL v2.

#include "c_agibot_hand_rs.h"
#include <cstring>
#include <iostream>
#include "rs_485_device/crc16.h"

#define CANID_WRITE_FLAG 0x01
#define CANID_READ_FLAG 0x00
#define CANID_PRODUCT_ID 0x01

#define DEGREE_OF_FREEDOM 10
#define CHECK_TIME 500

namespace YAML {
template <>
struct convert<AgibotHandRsO10::Options> {
  static bool decode(const Node& node, AgibotHandRsO10::Options& options) {
    if (!node.IsMap()) return false;
    if (node["uart_port"]) {
      options.uart_port = node["uart_port"].as<std::string>();
    }
    if (node["uart_baudrate"]) {
      options.uart_baudrate = node["uart_baudrate"].as<int32_t>();
    }
    return true;
  }
};
}  // namespace YAML

AgibotHandRsO10::AgibotHandRsO10(const YAML::Node& options_node) {
  Options options;
  if (options_node && !options_node.IsNull()) {
    options = options_node.as<Options>();
  }

  handrs485_interface_ =
      std::make_unique<UartRs485Interface>(options.uart_port, options.uart_baudrate);
  handrs485_interface_->InitDevice();
  // uint8_t check_cmd[] = {0xEE, 0xAA, 0x01, 0x00, 0x01, 0xCD, 0x55, 0x55};
  // handrs485_interface_->WriteDevice(check_cmd, sizeof(check_cmd)); //for debug the uart-485 connection
}

void AgibotHandRsO10::SetJointMotorPosi(unsigned char joint_motor_index, int16_t posi) {
  if (joint_motor_index > 10 || joint_motor_index < 1) {
    printf("the joint motor index input error, should be 1 -10, \n");
    return;
  }
  uint8_t setjoint_cmd[11] = {0xEE, 0xAA, 0x01, 0x00, 0x04, 0x6, 0x1, 0x33, 0x2, 0x55, 0x55};
  setjoint_cmd[6] = joint_motor_index;
  setjoint_cmd[7] = posi % 256;
  setjoint_cmd[8] = (posi >> 8) & 0xFF;
  uint16_t crc_val = Crc16(setjoint_cmd, 9);
  setjoint_cmd[9] = crc_val % 256;
  setjoint_cmd[10] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(setjoint_cmd, sizeof(setjoint_cmd));
  return;
}

int16_t AgibotHandRsO10::GetJointMotorPosi(unsigned char joint_motor_index) {
  if (joint_motor_index > 10 || joint_motor_index < 1) {
    printf("the joint index input error, should be 1 -10, \n");
    return 0;
  }
  uint8_t getjoint_cmd[9] = {0xEE, 0xAA, 0x01, 0x00, 0x04, 0x7, 0x1, 0x55, 0x55};
  getjoint_cmd[4] = 2;
  getjoint_cmd[5] = 7;
  getjoint_cmd[6] = joint_motor_index;
  uint16_t crc_val = Crc16(getjoint_cmd, 7);
  getjoint_cmd[7] = crc_val % 256;
  getjoint_cmd[8] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(getjoint_cmd, sizeof(getjoint_cmd));
  uint8_t check_time = CHECK_TIME;
  while (check_time--) {
    usleep(100000);
    if (handrs485_interface_->getjointmotorposi_feedback_state_) {
      handrs485_interface_->getjointmotorposi_feedback_state_ = 0;
      return handrs485_interface_->getjointmotorposi_result_;
    }
  }
  printf("get joint motor posi failed, joint_motor_index: %d\n", joint_motor_index);
  return 0;
}

void AgibotHandRsO10::SetAllJointMotorPosi(std::vector<int16_t> vec_posi) {
  if (vec_posi.size() < 10) {
    printf("joint number should not less than 10\n");
    return;
  }
  uint8_t setalljoint_cmd[28] = {0};
  setalljoint_cmd[0] = 0xEE;
  setalljoint_cmd[1] = 0xAA;
  setalljoint_cmd[2] = 0x1;
  setalljoint_cmd[3] = 0x0;
  setalljoint_cmd[4] = 0x15;  // 21 bytes
  setalljoint_cmd[5] = 0x8;

  for (int i = 0; i < 10; ++i) {
    uint16_t pos = static_cast<uint16_t>(vec_posi[i]);
    setalljoint_cmd[6 + 2 * i] = pos & 0xFF;    // low byte
    setalljoint_cmd[6 + 2 * i + 1] = pos >> 8;  // high byte
  }

  uint16_t crc_val = Crc16(setalljoint_cmd, 26);
  setalljoint_cmd[26] = crc_val % 256;
  setalljoint_cmd[27] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(setalljoint_cmd, sizeof(setalljoint_cmd));
  return;
}

std::vector<int16_t> AgibotHandRsO10::GetAllJointMotorPosi() {
  uint8_t getalljoint_cmd[8] = {0};
  getalljoint_cmd[0] = 0xEE;
  getalljoint_cmd[1] = 0xAA;
  getalljoint_cmd[2] = 0x1;
  getalljoint_cmd[3] = 0x0;
  getalljoint_cmd[4] = 0x1;  // 21 bytes
  getalljoint_cmd[5] = 0x9;
  uint16_t crc_val = Crc16(getalljoint_cmd, 6);
  getalljoint_cmd[6] = crc_val % 256;
  getalljoint_cmd[7] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(getalljoint_cmd, sizeof(getalljoint_cmd));
  uint8_t check_time = CHECK_TIME;
  while (check_time--) {  // check several times
    usleep(100000);       // 100ms
    if (handrs485_interface_->getalljointmotorposi_feedback_state_) {
      handrs485_interface_->getalljointmotorposi_feedback_state_ = 0;
      return handrs485_interface_->getalljointmotorposi_result_;
    }
  }
  printf("get all joint motor posi failed, joint_motor_index: \n");
  return handrs485_interface_->getalljointmotorposi_result_;
}
#if !DISABLE_FUNC
void AgibotHandCanO10::SetActiveJointAngle(unsigned char joint_motor_index, double angle) {
  return;
}

double AgibotHandRsO10::GetActiveJointAngle(unsigned char joint_motor_index) {
  return 0.0;
}
#endif

void AgibotHandRsO10::SetAllActiveJointAngles(std::vector<double> vec_angle) {
  if (vec_angle.size() != DEGREE_OF_FREEDOM) {
    std::cerr << "[Error]: 无效参数，需与主动自由度数量 " << std::dec << DEGREE_OF_FREEDOM << " 相匹配." << std::endl;
    return;
  }

  // 使用运动学求解器转换关节角度为电机位置
  std::vector<int> motor_positions = kinematics_solver_ptr_->ActiveJointPos2ActuatorInput(vec_angle);

  // 转换为short向量
  std::vector<int16_t> motor_posi_short(motor_positions.begin(), motor_positions.end());

  // 设置所有电机位置
  SetAllJointMotorPosi(motor_posi_short);
  return;
}

std::vector<double> AgibotHandRsO10::GetAllActiveJointAngles() {
  // 获取所有电机位置
  std::vector<int16_t> motor_posi = GetAllJointMotorPosi();

  // 转换为int向量供运动学求解器使用
  std::vector<int> motor_positions(motor_posi.begin(), motor_posi.end());

  // 使用运动学求解器转换电机位置为关节角度
  return kinematics_solver_ptr_->ActuatorInput2ActiveJointPos(motor_positions);
}

std::vector<double> AgibotHandRsO10::GetAllJointAngles() {
  std::vector<double> active_joint_angles = GetAllActiveJointAngles();
  return kinematics_solver_ptr_->GetAllJointPos(active_joint_angles);
}

#if !DISABLE_FUNC
void AgibotHandCanO10::SetJointMotorTorque(unsigned char joint_motor_index, int16_t torque) {
  return;
}

int16_t AgibotHandCanO10::GetJointMotorTorque(unsigned char joint_motor_index) {
  return 0;
}

void AgibotHandCanO10::SetAllJointMotorTorque(std::vector<int16_t> vec_torque) {
  return;
}

std::vector<int16_t> AgibotHandCanO10::GetAllJointMotorTorque() {
  return {};
}
#endif

void AgibotHandRsO10::SetJointMotorVelo(unsigned char joint_motor_index, int16_t velo) {
  return;
}

int16_t AgibotHandRsO10::GetJointMotorVelo(unsigned char joint_motor_index) {
  return 0;
}

void AgibotHandRsO10::SetAllJointMotorVelo(std::vector<int16_t> vec_velo) {
  if (vec_velo.size() < 10) {
    printf("velocity data number should not less than 10\n");
    return;
  }
  uint8_t setalljointvelo_cmd[28] = {0};
  setalljointvelo_cmd[0] = 0xEE;
  setalljointvelo_cmd[1] = 0xAA;
  setalljointvelo_cmd[2] = 0x1;
  setalljointvelo_cmd[3] = 0x0;
  setalljointvelo_cmd[4] = 0x15;  // 21 bytes
  setalljointvelo_cmd[5] = 0x20;

  for (int i = 0; i < 10; ++i) {
    uint16_t pos = static_cast<uint16_t>(vec_velo[i]);
    setalljointvelo_cmd[6 + 2 * i] = pos & 0xFF;    // low byte
    setalljointvelo_cmd[6 + 2 * i + 1] = pos >> 8;  // high byte
  }

  uint16_t crc_val = Crc16(setalljointvelo_cmd, 26);
  setalljointvelo_cmd[26] = crc_val % 256;
  setalljointvelo_cmd[27] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(setalljointvelo_cmd, sizeof(setalljointvelo_cmd));
  return;
}

std::vector<int16_t> AgibotHandRsO10::GetAllJointMotorVelo() {
  uint8_t getalljointvelo_cmd[8] = {0};
  getalljointvelo_cmd[0] = 0xEE;
  getalljointvelo_cmd[1] = 0xAA;
  getalljointvelo_cmd[2] = 0x1;
  getalljointvelo_cmd[3] = 0x0;
  getalljointvelo_cmd[4] = 0x1;  // 21 bytes
  getalljointvelo_cmd[5] = 0xB;
  uint16_t crc_val = Crc16(getalljointvelo_cmd, 6);
  getalljointvelo_cmd[6] = crc_val % 256;
  getalljointvelo_cmd[7] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(getalljointvelo_cmd, sizeof(getalljointvelo_cmd));
  uint8_t check_time = CHECK_TIME;
  while (check_time--) {  // check several times
    usleep(100000);       // 100ms
    if (handrs485_interface_->getalljointmotorvelo_feedback_state_) {
      handrs485_interface_->getalljointmotorvelo_feedback_state_ = 0;
      return handrs485_interface_->getalljointmotorvelo_result_;
    }
  }
  printf("get all joint motor velocity failed, joint_motor_index: \n");
  return handrs485_interface_->getalljointmotorvelo_result_;
}

std::vector<uint8_t> AgibotHandRsO10::GetTactileSensorData(EFinger eFinger) {
  if (eFinger == EFinger::eUnknown ||
      static_cast<unsigned char>(eFinger) < static_cast<unsigned char>(EFinger::eThumb) ||
      static_cast<unsigned char>(eFinger) > static_cast<unsigned char>(EFinger::eDorsum)) {
    throw std::invalid_argument("Invalid finger type");
  }

  uint8_t getsensordata_cmd[9] = {0xEE, 0xAA, 0x01, 0x00, 0x04, 0x7, 0x1, 0x55, 0x55};
  getsensordata_cmd[4] = 2;
  getsensordata_cmd[5] = 0x11;
  getsensordata_cmd[6] = (uint8_t)eFinger - 1;
  uint16_t crc_val = Crc16(getsensordata_cmd, 7);
  getsensordata_cmd[7] = crc_val % 256;
  getsensordata_cmd[8] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(getsensordata_cmd, sizeof(getsensordata_cmd));
  uint8_t check_time = CHECK_TIME;
  while (check_time--) {
    if (handrs485_interface_->getsensordata_feedback_state_) {
      const size_t FINGER_DATA_LENGTH = 16;
      const size_t PALM_DATA_LENGTH = 25;
      std::vector<uint8_t> result;
      if (eFinger == EFinger::eDorsum || eFinger == EFinger::ePalm) {
        result.resize(PALM_DATA_LENGTH);
        memcpy(result.data(), handrs485_interface_->getsensordata_result_.data(),
               PALM_DATA_LENGTH * sizeof(uint8_t));
      } else {
        result.resize(FINGER_DATA_LENGTH);
        memcpy(result.data(), handrs485_interface_->getsensordata_result_.data(),
               FINGER_DATA_LENGTH * sizeof(uint8_t));
      }

      handrs485_interface_->getsensordata_feedback_state_ = 0;
      return result;
    }
    usleep(1000);
  }
  printf("get joint sensor data failed, finger type: %d\n", static_cast<uint8_t>(eFinger));
  return {};
}

void AgibotHandRsO10::SetControlMode(unsigned char joint_motor_index, EControlMode mode) {
  if (joint_motor_index > 10 || joint_motor_index < 1) {
    printf("the joint motor index input error, should be 1 -10, \n");
    return;
  }
  uint8_t setcontrol_cmd[10] = {0xEE, 0xAA, 0x01, 0x00, 0x04, 0x15, 0x1, 0x33, 0x55, 0x55};
  setcontrol_cmd[6] = joint_motor_index;
  setcontrol_cmd[7] = (uint8_t)mode;
  uint16_t crc_val = Crc16(setcontrol_cmd, 8);
  setcontrol_cmd[8] = crc_val % 256;
  setcontrol_cmd[9] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(setcontrol_cmd, sizeof(setcontrol_cmd));
  return;  // 0x15 need test
}

EControlMode AgibotHandRsO10::GetControlMode(unsigned char joint_motor_index) {
  return EControlMode::eUnknown;
}

void AgibotHandRsO10::SetAllControlMode(std::vector<unsigned char> vec_ctrl_mode) {
  return;
}

std::vector<unsigned char> AgibotHandRsO10::GetAllControlMode() {
  return {};
}

void AgibotHandRsO10::SetCurrentThreshold(unsigned char joint_motor_index, int16_t current_threshold) {
  return;
}

int16_t AgibotHandRsO10::GetCurrentThreshold(unsigned char joint_motor_index) {
  return {};
}

void AgibotHandRsO10::SetAllCurrentThreshold(std::vector<int16_t> vec_current_threshold) {
  return;
}

std::vector<int16_t> AgibotHandRsO10::GetAllCurrentThreshold() {
  return {};
}

void AgibotHandRsO10::MixCtrlJointMotor(std::vector<MixCtrl> vec_mix_ctrl) {
  return;
}

JointMotorErrorReport AgibotHandRsO10::GetErrorReport(unsigned char joint_motor_index) {
  if (joint_motor_index > 10 || joint_motor_index < 1) {
    printf("the joint motor index input error, should be 1 -10, \n");
    return {};
  }
  JointMotorErrorReport ret_error = {0};
  uint8_t geterrorport_cmd[8] = {0};
  geterrorport_cmd[0] = 0xEE;
  geterrorport_cmd[1] = 0xAA;
  geterrorport_cmd[2] = 0x1;
  geterrorport_cmd[3] = 0x0;
  geterrorport_cmd[4] = 0x1;
  geterrorport_cmd[5] = 0xD;
  uint16_t crc_val = Crc16(geterrorport_cmd, 6);
  geterrorport_cmd[6] = crc_val % 256;
  geterrorport_cmd[7] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(geterrorport_cmd, sizeof(geterrorport_cmd));
  uint8_t check_time = CHECK_TIME;
  while (check_time--) {  // check several times
    usleep(100000);       // 100ms
    if (handrs485_interface_->getallerrorreport_feedback_state_) {
      uint16_t check_res = handrs485_interface_->getallerrorreport_result_.res_[0] + handrs485_interface_->getallerrorreport_result_.res_[1] * 256;
      if (check_res > 0 && check_res < 11) {
        ret_error.stalled_ = check_res;
      } else if (check_res > 20 && check_res < 31) {
        ret_error.overheat_ = check_res;
      } else if (check_res > 40 && check_res < 51) {
        ret_error.over_current_ = check_res - 40;
      } else if (check_res > 60 && check_res < 71) {
        ret_error.motor_except_ = check_res - 60;
      } else if (check_res == 101) {
        ret_error.commu_except_ = check_res;
      }
      // return handrs485_interface_->getallerrorreport_result_;
      handrs485_interface_->getallerrorreport_feedback_state_ = 0;
      return ret_error;
    }
  }
  printf("get error report failed, joint_motor_index: %d\n", joint_motor_index);
  return ret_error;
  ;
  // need test
}

std::vector<JointMotorErrorReport> AgibotHandRsO10::GetAllErrorReport() {
  std::vector<JointMotorErrorReport> all_errorreport;
  uint8_t geterrorport_cmd[8] = {0};
  geterrorport_cmd[0] = 0xEE;
  geterrorport_cmd[1] = 0xAA;
  geterrorport_cmd[2] = 0x1;
  geterrorport_cmd[3] = 0x0;
  geterrorport_cmd[4] = 0x1;
  geterrorport_cmd[5] = 0xD;
  uint16_t crc_val = Crc16(geterrorport_cmd, 6);
  geterrorport_cmd[6] = crc_val % 256;
  geterrorport_cmd[7] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(geterrorport_cmd, sizeof(geterrorport_cmd));
  uint8_t check_time = CHECK_TIME;
  while (check_time--) {  // check several times
    usleep(100000);       // 100ms
    if (handrs485_interface_->getallerrorreport_feedback_state_) {
      uint16_t check_res = handrs485_interface_->getallerrorreport_result_.res_[0] + handrs485_interface_->getallerrorreport_result_.res_[1] * 256;
      if (check_res > 0 && check_res < 11) {
        all_errorreport[check_res - 10].stalled_ = check_res;
      } else if (check_res > 20 && check_res < 31) {
        all_errorreport[check_res - 20].overheat_ = check_res;
      } else if (check_res > 40 && check_res < 51) {
        all_errorreport[check_res - 40].over_current_ = check_res - 40;
      } else if (check_res > 60 && check_res < 71) {
        all_errorreport[check_res - 60].motor_except_ = check_res - 60;
      }
      // return handrs485_interface_->getallerrorreport_result_;
      handrs485_interface_->getallerrorreport_feedback_state_ = 0;
      return all_errorreport;
    }
  }
  printf("get error report failed\n");
  return all_errorreport;  // need test
}
#if !DISABLE_FUNC
void AgibotHandRsO10::SetErrorReportPeriod(unsigned char joint_motor_index, uint16_t period) {
  return;
}

void AgibotHandRsO10::SetAllErrorReportPeriod(std::vector<uint16_t> vec_period) {
  return;
}
#endif
uint16_t AgibotHandRsO10::GetTemperatureReport(unsigned char joint_motor_index) {
  if (joint_motor_index > 10 || joint_motor_index < 1) {
    printf("the joint motor index input error, should be 1 -10, \n");
    return 0;
  }
  uint8_t gettempreport_cmd[8] = {0xEE, 0xAA, 0x01, 0x00, 0x04, 0x7, 0x55, 0x55};
  gettempreport_cmd[4] = 1;
  gettempreport_cmd[5] = 0xC;
  uint16_t crc_val = Crc16(gettempreport_cmd, 6);
  gettempreport_cmd[6] = crc_val % 256;
  gettempreport_cmd[7] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(gettempreport_cmd, sizeof(gettempreport_cmd));
  uint8_t check_time = CHECK_TIME;
  while (check_time--) {
    usleep(100000);
    if (handrs485_interface_->getalltempreport_feedback_state_) {
      handrs485_interface_->getalltempreport_feedback_state_ = 0;
      return handrs485_interface_->getalltempreport_result_[joint_motor_index - 1];
    }
  }
  printf("get motor temprature failed, joint_motor_index: %d\n", joint_motor_index);
  return 0;  // 0xC
}

std::vector<uint16_t> AgibotHandRsO10::GetAllTemperatureReport() {
  std::vector<uint16_t> alltempresult(DEGREE_OF_FREEDOM, 0);
  uint8_t getalltempreport_cmd[8] = {0xEE, 0xAA, 0x01, 0x00, 0x04, 0x7, 0x55, 0x55};
  getalltempreport_cmd[4] = 1;
  getalltempreport_cmd[5] = 0xC;
  uint16_t crc_val = Crc16(getalltempreport_cmd, 6);
  getalltempreport_cmd[6] = crc_val % 256;
  getalltempreport_cmd[7] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(getalltempreport_cmd, sizeof(getalltempreport_cmd));
  uint8_t check_time = CHECK_TIME;
  while (check_time--) {
    usleep(100000);
    if (handrs485_interface_->getalltempreport_feedback_state_) {
      memcpy(alltempresult.data(), handrs485_interface_->getalltempreport_result_, DEGREE_OF_FREEDOM * sizeof(uint16_t));

      handrs485_interface_->getalltempreport_feedback_state_ = 0;
      return alltempresult;
    }
  }
  printf("get all motor temprature failed\n");
  return alltempresult;  // 0xC
}
#if !DISABLE_FUNC
void AgibotHandRsO10::SetTemperReportPeriod(unsigned char joint_motor_index, uint16_t period) {
  return;
}

void AgibotHandRsO10::SetAllTemperReportPeriod(std::vector<uint16_t> vec_period) {
  return;
}
#endif
int16_t AgibotHandRsO10::GetCurrentReport(unsigned char joint_motor_index) {
  if (joint_motor_index > 10 || joint_motor_index < 1) {
    printf("the joint motor index input error, should be 1 -10, \n");
    return 0;
  }
  uint16_t current_res = 0;
  uint8_t getcurrent_cmd[8] = {0xEE, 0xAA, 0x01, 0x00, 0x04, 0x7, 0x55, 0x55};
  getcurrent_cmd[4] = 1;
  getcurrent_cmd[5] = 0xA;
  uint16_t crc_val = Crc16(getcurrent_cmd, 6);
  getcurrent_cmd[6] = crc_val % 256;
  getcurrent_cmd[7] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(getcurrent_cmd, sizeof(getcurrent_cmd));
  uint8_t check_time = CHECK_TIME;
  while (check_time--) {
    usleep(100000);
    if (handrs485_interface_->getallcurrentreport_feedback_state_) {
      handrs485_interface_->getallcurrentreport_feedback_state_ = 0;
      return handrs485_interface_->getallcurrentreport_result_[joint_motor_index - 1];
    }
  }
  printf("get motor current failed, joint_motor_index: %d\n", joint_motor_index);
  return handrs485_interface_->getallcurrentreport_result_[joint_motor_index - 1];  // 0xA
}

std::vector<uint16_t> AgibotHandRsO10::GetAllCurrentReport() {
  std::vector<uint16_t> allcurrentresult(DEGREE_OF_FREEDOM, 0);
  uint8_t getallcurrent_cmd[8] = {0xEE, 0xAA, 0x01, 0x00, 0x04, 0x7, 0x55, 0x55};
  getallcurrent_cmd[4] = 1;
  getallcurrent_cmd[5] = 0xA;
  uint16_t crc_val = Crc16(getallcurrent_cmd, 6);
  getallcurrent_cmd[6] = crc_val % 256;
  getallcurrent_cmd[7] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(getallcurrent_cmd, sizeof(getallcurrent_cmd));
  uint8_t check_time = CHECK_TIME;
  while (check_time--) {
    usleep(100000);
    if (handrs485_interface_->getallcurrentreport_feedback_state_) {
      memcpy(allcurrentresult.data(), handrs485_interface_->getallcurrentreport_result_, DEGREE_OF_FREEDOM * sizeof(uint16_t));
      handrs485_interface_->getallcurrentreport_feedback_state_ = 0;
      return allcurrentresult;
    }
  }
  printf("get all motor current failed.\n");
  return allcurrentresult;  // 0xA
}
#if !DISABLE_FUNC
void AgibotHandRsO10::SetCurrentReportPeriod(unsigned char joint_motor_index, uint16_t period) {
  return;
}

void AgibotHandRsO10::SetAllCurrentReportPeriod(std::vector<uint16_t> vec_period) {
  return;
}
#endif
VendorInfo AgibotHandRsO10::GetVendorInfo() {
  uint8_t getvendorinfo_cmd[8] = {0xEE, 0xAA, 0x01, 0x00, 0x04, 0x7, 0x55, 0x55};
  getvendorinfo_cmd[4] = 1;
  getvendorinfo_cmd[5] = 0xCD;
  uint16_t crc_val = Crc16(getvendorinfo_cmd, 6);
  getvendorinfo_cmd[6] = crc_val % 256;
  getvendorinfo_cmd[7] = (crc_val >> 8) & 0xFF;
  handrs485_interface_->WriteDevice(getvendorinfo_cmd, sizeof(getvendorinfo_cmd));
  uint8_t check_time = CHECK_TIME;
  while (check_time--) {
    usleep(100000);
    if (handrs485_interface_->getvendorinfo_feedback_state_) {
      handrs485_interface_->getvendorinfo_feedback_state_ = 0;
      return handrs485_interface_->getvendorinfo_result_;
    }
  }
  printf("get vendor info failed.\n");
  return handrs485_interface_->getvendorinfo_result_;  // 0xCD
}

DeviceInfo AgibotHandRsO10::GetDeviceInfo() {
  return {};
}

void AgibotHandRsO10::SetDeviceId(unsigned char device_id) {
  return;
}

void AgibotHandRsO10::ShowDataDetails(bool show) const {
  handrs485_interface_->ShowDataDetails(show);
  kinematics_solver_ptr_->show_log(show);
  return;
}