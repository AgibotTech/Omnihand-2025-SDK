// main.cc
// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand 2025 SDK is licensed under Mulan PSL v2.

#include <chrono>
#include <iostream>
#include <thread>
#include "c_agibot_hand_base.h"

void positionControlDemo() {
  try {
    auto hand = AgibotHandO10::createHand(1, 0, EHandType::eLeft);

    // // temperature （9 10 索引温一直是0 请查清楚）
    // auto tm = hand->GetTemperatureReport(8);
    // std::cout << "8 temp: " << tm << std::endl;

    // auto tms = hand->GetAllTemperatureReport();
    // int idx = 1;
    // for (auto c : tms) {
    //   std::cout << "temp " << idx++ << " : " << c << std::endl;
    // }

    // // current
    // auto ct = hand->GetCurrentReport(8);
    // std::cout << "8 current: " << ct << std::endl;

    // auto cts = hand->GetAllCurrentReport();
    // idx = 1;
    // for (auto c : cts) {
    //   std::cout << "current " << idx++ << " : " << c << std::endl;
    // }

    // // error report (这个案例输出什么都没有，是空)
    // JointMotorErrorReport ep = hand->GetErrorReport(8);
    // std::cout << "8 error report : " << ep.overheat_ << std::endl;

    // auto eps = hand->GetAllErrorReport();
    // idx = 1;
    // for (auto c : eps) {
    //   std::cout << "error report " << idx++ << " : " << c.overheat_ << std::endl;
    // }

    // // sensor
    // std::vector<uint8_t> a = hand->GetTactileSensorData(EFinger::eThumb);
    // int sum = 0;
    // for (auto& c : a) {
    //   sum += c;
    // }
    // std::cout << a.size() << "拇指传感器数据: " << sum << std::endl;

    // std::vector<uint8_t> b = hand->GetTactileSensorData(EFinger::ePalm);
    // sum = 0;
    // for (auto& c : b) {
    //   sum += c;
    // }
    // std::cout << b.size() << "掌心传感器数据: " << sum << std::endl;


    std::vector<int16_t> vec_pos1 {500, 2081, 4094, 2029, 4094, 4094, 2048, 4094, 4000, 4094};
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    hand->SetAllJointMotorPosi(vec_pos1);

    std::vector<int16_t> vec_pos2 {2000, 2081, 4094, 2029, 4094, 4094, 2048, 4094, 4000, 4094};
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    hand->SetAllJointMotorPosi(vec_pos2);

    std::vector<int16_t> vec_pos3 {500, 2081, 4094, 2029, 4094, 4094, 2048, 4094, 4000, 4094};
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    hand->SetAllJointMotorPosi(vec_pos3);

    std::vector<int16_t> vec_pos4 {500, 2081, 4094, 2029, 4094, 4094, 2048, 4094, 4000, 4094};
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    hand->SetAllJointMotorPosi(vec_pos4);
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