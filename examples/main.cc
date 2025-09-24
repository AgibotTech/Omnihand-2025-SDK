// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand 2025 SDK is licensed under Mulan PSL v2.

#include <chrono>
#include <iostream>
#include <thread>
#include "c_agibot_hand.h"

void positionControlDemo() {
  try {
    AgibotHandO10 hand(1, EHandType::eLeft);

    // 设置单个关节位置
    hand.SetJointMotorPosi(8, 1000);
    std::cout << "设置关节10位置为1000" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 获取单个关节位置
    short pos = hand.GetJointMotorPosi(8);
    std::cout << "关节8当前位置: " << pos << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 批量设置所有关节位置
    std::vector<short> positions = {4096, 316, 1179, 0, 2470, 3731, 2409, 3832, 2307, 3922};
    hand.SetAllJointMotorPosi(positions);
    std::cout << "批量设置所有关节位置完成" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // 批量获取所有关节位置
    std::vector<short> allPositions = hand.GetAllJointMotorPosi();
    std::cout << "所有关节位置: ";
    for (size_t i = 0; i < allPositions.size(); ++i) {
      std::cout << allPositions[i];
      if (i < allPositions.size() - 1) std::cout << ", ";
    }
    std::cout << std::endl;

    // temperature （9 10 索引温一直是0 请查清楚）
    auto tm = hand.GetTemperatureReport(8);
    std::cout << "8 temp: " << tm << std::endl;

    auto tms = hand.GetAllTemperatureReport();
    int idx = 1;
    for (auto c : tms) {
      std::cout << "temp " << idx++ << " : " << c << std::endl;
    }

    // current
    auto ct = hand.GetCurrentReport(8);
    std::cout << "8 current: " << ct << std::endl;

    auto cts = hand.GetAllCurrentReport();
    idx = 1;
    for (auto c : cts) {
      std::cout << "current " << idx++ << " : " << c << std::endl;
    }

    // error report (这个案例输出什么都没有，是空)
    JointMotorErrorReport ep = hand.GetErrorReport(8);
    std::cout << "8 error report : " << ep.motor_except_ << std::endl;

    auto eps = hand.GetAllErrorReport();
    idx = 1;
    for (auto c : eps) {
      std::cout << "error report " << idx++ << " : " << c.commu_except_ << std::endl;
    }

    // sensor
    std::vector<uint8_t> a = hand.GetTouchSensorData(EFinger::eThumb);
    int sum = 0;
    for (auto& c : a) {
      sum += c;
    }
    std::cout << a.size() << "拇指传感器数据: " << sum << std::endl;

    std::vector<uint8_t> b = hand.GetTouchSensorData(EFinger::ePalm);
    sum = 0;
    for (auto& c : b) {
      sum += c;
    }
    std::cout << b.size() << "掌心传感器数据: " << sum << std::endl;

  } catch (const std::exception& e) {
    std::cerr << "错误: " << e.what() << std::endl;
  }
}

int main() {
  std::cout << "OmniHand 2025 C++ SDK 功能演示" << std::endl;

  positionControlDemo();

  std::cout << "演示完成!" << std::endl;

  return 0;
}