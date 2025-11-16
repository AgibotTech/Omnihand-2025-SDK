// main.cc
// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand 2025 SDK is licensed under Mulan PSL v2.

#include <chrono>
#include <iostream>
#include <thread>
#include "c_agibot_hand_base.h"

void ControlHand(std::unique_ptr<AgibotHandO10>& hand) {
  std::cout << "Begin ControlHand" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(2000));
  std::vector<int16_t> vec_pos1 {500, 2081, 4094, 2029, 4094, 4094, 2048, 4094, 4000, 4094};
  hand->SetAllJointMotorPosi(vec_pos1);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::vector<int16_t> vec_pos2 {2000, 2081, 4094, 2029, 4094, 4094, 2048, 4094, 4000, 4094};
  hand->SetAllJointMotorPosi(vec_pos2);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::vector<int16_t> vec_pos3 {500, 2081, 4094, 2029, 4094, 4094, 2048, 4094, 4000, 4094};
  hand->SetAllJointMotorPosi(vec_pos3);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  std::vector<int16_t> vec_pos4 {1500, 2081, 4094, 2029, 4094, 4094, 2048, 4094, 4000, 4094};
  hand->SetAllJointMotorPosi(vec_pos4);
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
}

void GetHandInfo(std::unique_ptr<AgibotHandO10>& hand) {
  std::cout << "Begin GetHandInfo" << std::endl;
  int index = 0;
  while (true) {
    auto pos = hand->GetAllJointMotorPosi();
    if (index % 1000 == 0) {
      std::string joint_pos_str;
      for (const auto& p : pos) {
        joint_pos_str += std::to_string(p) + " ";
      }
      std::cout << "关节位置: [" << joint_pos_str << "]" << std::endl;
    }
    
    std::this_thread::sleep_for(std::chrono::milliseconds(1));


    if (index / 1000 == 5) {
      break;
    }
    index++;
  }
}


void positionControlDemo() {
  try {
    auto hand = AgibotHandO10::createHand(1, 0, EHandType::eLeft);

    std::thread control_thread(ControlHand, std::ref(hand));
    std::thread info_thread(GetHandInfo, std::ref(hand));

    control_thread.join();
    info_thread.join();

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