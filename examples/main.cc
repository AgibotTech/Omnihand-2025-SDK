// main.cc
// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand 2025 SDK is licensed under Mulan PSL v2.

#include <chrono>
#include <iostream>
#include <thread>
#include "c_agibot_hand_base.h"

void positionControlDemo() {
  // 使用工厂方法创建实例
  auto hand = AgibotHandO10::createHand(
      0x01,               // device_id = 1
      EHandType::eRight,  // 左手,
      "./conf/hardware_conf.yaml");

  // 设置单个关节位置
  hand->SetJointMotorPosi(8, 1000);
  std::cout << "设置关节10位置为1000" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // 获取单个关节位置
  int16_t pos = hand->GetJointMotorPosi(8);
  std::cout << "关节8当前位置: " << pos << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(2));

  // 批量设置所有关节位置
  std::vector<int16_t> positions = {4096, 316, 1179, 0, 2470, 3731, 2409, 3832, 2307, 3922};
  hand->SetAllJointMotorPosi(positions);
  std::cout << "批量设置所有关节位置完成" << std::endl;
  std::this_thread::sleep_for(std::chrono::seconds(1));

  // 批量获取所有关节位置
  std::vector<int16_t> allPositions = hand->GetAllJointMotorPosi();
  std::cout << "所有关节位置: ";
  for (size_t i = 0; i < allPositions.size(); ++i) {
    std::cout << allPositions[i];
    if (i < allPositions.size() - 1) std::cout << ", ";
  }
  std::cout << std::endl;
}

int main() {
  std::cout << "OmniHand 2025 C++ SDK 功能演示" << std::endl;

  positionControlDemo();
  std::cout << "演示完成!" << std::endl;

  return 0;
}