// Copyright (c) 2025, Agibot Co., Ltd.
// OmniHand 2025 SDK is licensed under Mulan PSL v2.

/**
 * @file c_can_bus_device.cpp
 * @brief
 * @author AgiUser
 * @date 25-7-27
 **/

#include "c_can_bus_device.h"

#include <iostream>
#include <utility>

void CanBusDeviceBase::SetMsgMatchJudge(std::function<bool(unsigned int, unsigned int)> msg_matched_judge) {
  msg_match_judge_ = std::move(msg_matched_judge);
}

CanfdFrame CanBusDeviceBase::SendRequestSynch(const CanfdFrame& req) {
  /*发送请求帧*/
  {
    std::lock_guard<std::mutex> lockGuard(mutex_reqRep_);
    if (SendFrame(req.can_id_, (unsigned char*)req.data_, req.len_) == -1) {
      std::stringstream sstream;
      sstream << "[ERROR]: CANFD ID: 0x" << std::hex << std::setw(8) << std::setfill('0')
              << (req.can_id_ & 0x1FFFFFFF) << " 请求发送失败" << std::endl;
      throw std::runtime_error(sstream.str());
    }

    uset_req_.insert(req.can_id_);
    // 时序问题可能导致已经发送请求并接收到应答报文后，set还未更新导致应答无法匹配转向回调，请求没有匹配到应答就会阻塞提示超时
  }

  if (show_data_details_.load()) {
    auto now = std::chrono::system_clock::now();
    auto now_time_t = std::chrono::system_clock::to_time_t(now);

    // 获取微秒
    auto now_us = std::chrono::duration_cast<std::chrono::microseconds>(now.time_since_epoch()) % 1000000;
    struct tm tm_buf;
    localtime_r(&now_time_t, &tm_buf);
    std::cout << "["
              << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S")
              << "." << std::dec << std::setfill('0') << std::setw(6) << now_us.count()
              << "] SND: " << req;
  }

  auto start = std::chrono::high_resolution_clock::now();
  while (true) {
    {
      std::lock_guard<std::mutex> lockGuard(mutex_reqRep_);
      auto iter = umap_rep_.find(calcu_match_rep_id_(req.can_id_));
      if (iter != umap_rep_.end()) {
        uset_req_.erase(req.can_id_);
        CanfdFrame rep = iter->second;
        umap_rep_.erase(iter);
        return rep;
      }
    }

    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    if (duration.count() > 50) {
      uset_req_.erase(req.can_id_);
      std::stringstream sstream;
      sstream << "[ERROR]: CANFD ID: 0x" << std::hex << std::setw(8) << std::setfill('0')
              << (req.can_id_ & 0x1FFFFFFF) << " 请求超时" << std::endl;
      throw std::runtime_error(sstream.str());
    }
  }

  std::cout << std::dec << std::endl;  // 恢复十进制输出
}

int CanBusDeviceBase::SendRequestWithoutReply(const CanfdFrame& req) {
  return SendFrame(req.can_id_, (unsigned char*)req.data_, req.len_);
}
