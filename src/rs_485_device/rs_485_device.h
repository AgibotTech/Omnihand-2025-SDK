#pragma once

#include <thread>
#include <vector>
#include "serial/serial.h"
#include "serial/v8stdint.h"
#define REC_BUF_LEN 1024
#define MIN_RESULT_LEN 8

class UartRs485Interface {
 public:
  /**
   * @brief
   */

  UartRs485Interface();

  /**
   * @brief
   */
  ~UartRs485Interface();

  /**
   * @open
   */
  void open(const std::string& dev,
            uint32_t baud = 460800,
            const serial::Timeout& timeout =
                serial::Timeout::simpleTimeout(2));

  /**
   * @brief InitDevice
   * @return
   */
  void InitDevice();

  /**
   * @brief WriteDevice
   * @return
   */
  uint8_t WriteDevice(uint8_t* data, uint8_t size);

  /**
   * @brief ReadDevice
   * @return
   */
  uint8_t ReadDevice(uint8_t* buf, uint8_t size);

  /**
   * @brief ThreadReadRec
   * @return
   */
  void ThreadReadRec(void);

  /**
   * @brief RecBuffParse
   * @return
   */
  void RecBuffParse(void);

  /**
   * @brief Rs485_device_ptr_
   * @return
   */
  serial::Serial Rs485_device_ptr_;

  /**
   * @brief   uint8_t getjointmotorposi_feedback_state_
   * @return
   */
  uint8_t getjointmotorposi_feedback_state_ = 0;
  /**
   * @brief   getjointmotorposi_result_
   * @return
   */
  uint16_t getjointmotorposi_result_ = 0;
  /**
   * @brief   getalljointmotorposi_feedback_state_
   * @return
   */
  uint8_t getalljointmotorposi_feedback_state_ = 0;
  /**
   * @brief   getalljointmotorposi_result_
   * @return
   */
  std::vector<int16_t> getalljointmotorposi_result_ = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // to get the hand motor position

 private:
  std::thread serial_rec_pthread_;
  uint8_t rec_buffer_[REC_BUF_LEN] = {0};
  uint16_t buf_write_pos_ = 0;
};
