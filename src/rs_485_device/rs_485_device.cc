#include "rs_485_device/rs_485_device.h"
#include <cstring>
#include <fstream>
#include <iostream>
#define UART_PORT "/dev/ttyUSB0"
#define UART_BAUD 460800

#define READ_BYTES_LIMIT 256
#define DOUBLE_CHECK 128

#define CMD_VER_REQ 0xCD
#define CMD_GET_JOINT_MOTOR_POSI 0x07
#define CMD_GET_ALL_JOINT_MOTOR_POSI 0x9
#define CMD_GET_ALL_JOINT_MOTOR_VELO 0xB
#define CMD_GET_SENSOR_DATA 0x11
#define CMD_GET_ALL_ERROR_REPORT 0x0D
#define CMD_GET_ALL_TEMP_REPORT 0x0C
#define CMD_GET_ALL_CURRENT_REPORT 0x0A
#define CMD_GET_VENDOR_INFO 0xCD

UartRs485Interface::UartRs485Interface()
    : Rs485_device_ptr_(UART_PORT, UART_BAUD,
                        serial::Timeout::simpleTimeout(2)) {
}

UartRs485Interface::~UartRs485Interface() {
  running_ = false;
  if (serial_rec_pthread_.joinable())
    serial_rec_pthread_.join();
}

void UartRs485Interface::RecBuffParse(void) {
  // printf("RecBuffParse: now data len is %d\n", buf_write_pos_); //for debug
  uint16_t detect_frame_len = 0;
  uint16_t index = 0;
  uint8_t temp[128] = {0};
  uint16_t hold_len = 0;
  for (int index = 0; (buf_write_pos_ >= MIN_RESULT_LEN) && (index <= (buf_write_pos_ - MIN_RESULT_LEN)); index++) {  // 8 is the shortest length for a feedback, 0xEE, 0xAA, 0x01,0x0, len, cmd crc1, crc2

    if (rec_buffer_[index] == 0xEE && rec_buffer_[index + 1] == 0xAA && rec_buffer_[index + 3] == 0) {
      uint8_t expect_complete_frame_end = index + rec_buffer_[index + 4] + 5 + 2;

      if (expect_complete_frame_end <= buf_write_pos_) {
        detect_frame_len += rec_buffer_[index + 4] + 5 + 2;
        switch (rec_buffer_[index + 5]) {
          // case CMD_VER_REQ:
          //   printf("version request response data ready !\n");
          //   break;
          case CMD_GET_JOINT_MOTOR_POSI:
            printf("get joint motor position response data ready !\n");
            getjointmotorposi_result_ = rec_buffer_[index + 7] + rec_buffer_[index + 8] * 256;
            getjointmotorposi_feedback_state_ = 1;
            break;
          case CMD_GET_ALL_JOINT_MOTOR_POSI:
            printf("get all joint motor position response data ready !\n");
            for (int i = 0; i < 10; i++) {
              getalljointmotorposi_result_.at(i) = rec_buffer_[index + 6 + 2 * i] + rec_buffer_[index + 6 + 2 * i + 1] * 256;
            }
            getalljointmotorposi_feedback_state_ = 1;
            break;

          case CMD_GET_ALL_JOINT_MOTOR_VELO:
            printf("get all joint motor velocity response data ready !\n");
            for (int i = 0; i < 10; i++) {
              getalljointmotorvelo_result_.at(i) = rec_buffer_[index + 6 + 2 * i] + rec_buffer_[index + 6 + 2 * i + 1] * 256;
            }
            getalljointmotorvelo_feedback_state_ = 1;
            break;

          case CMD_GET_SENSOR_DATA:
            printf("get sensor data response data ready! finger index : %d\n", rec_buffer_[index + 6]);

            memcpy(getsensordata_result_.data(), rec_buffer_ + index + 7, 25);

            getsensordata_feedback_state_ = 1;
            break;

          case CMD_GET_ALL_ERROR_REPORT:
            printf("get all error report response data ready! \n");
            getallerrorreport_result_.res_[0] = rec_buffer_[index + 6];
            getallerrorreport_result_.res_[0] = rec_buffer_[index + 7];
            getallerrorreport_feedback_state_ = 1;
            break;

          case CMD_GET_ALL_TEMP_REPORT:
            printf("get all temprature report response data ready! \n");
            for (int i = 0; i < 8; i++) {
              getalltempreport_result_[i] = rec_buffer_[index + 6 + i];
            }
            getalltempreport_feedback_state_ = 1;
            break;

          case CMD_GET_ALL_CURRENT_REPORT:
            printf("get all current report response data ready! \n");
            for (int i = 0; i < 10; i++) {
              // printf("all current data[%d]: %d \n", i, rec_buffer_[i]);
              getallcurrentreport_result_[i] = rec_buffer_[index + 6 + 2 * i] + rec_buffer_[index + 6 + 2 * i + 1] * 256;
            }
            getallcurrentreport_feedback_state_ = 1;
            break;

          case CMD_GET_VENDOR_INFO:
            printf("get vender info response data ready! \n");
            for (int i = 0; i < 10; i++) {
              // printf("vendor info data[%d]: %d \n", i, rec_buffer_[i]);
              getvendorinfo_result_.productModel = (rec_buffer_[index + 6] == 1) ? "O12" : "O10";
              getvendorinfo_result_.hardwareVersion.major_ = rec_buffer_[index + 12];
              getvendorinfo_result_.hardwareVersion.minor_ = rec_buffer_[index + 13];
              getvendorinfo_result_.hardwareVersion.patch_ = rec_buffer_[index + 14];

              getvendorinfo_result_.softwareVersion.major_ = rec_buffer_[index + 9];
              getvendorinfo_result_.softwareVersion.minor_ = rec_buffer_[index + 10];
              getvendorinfo_result_.softwareVersion.patch_ = rec_buffer_[index + 11];

              getvendorinfo_result_.dof = rec_buffer_[index + 15];
            }
            getvendorinfo_feedback_state_ = 1;
            break;

          default:
            break;
        }
      }
    }
  }
  if (detect_frame_len == 0) {
    hold_len = buf_write_pos_ > DOUBLE_CHECK ? DOUBLE_CHECK : buf_write_pos_;
  } else {
    uint16_t not_frame_len = buf_write_pos_ - detect_frame_len;
    hold_len = not_frame_len > DOUBLE_CHECK ? DOUBLE_CHECK : not_frame_len;
  }
  if (hold_len) {
    memcpy(&temp[0], &rec_buffer_[buf_write_pos_ - hold_len], hold_len);
    memset(rec_buffer_, 0, sizeof(rec_buffer_));
    memcpy(rec_buffer_, temp, hold_len);
    buf_write_pos_ = hold_len;
  }
}

void UartRs485Interface::ThreadReadRec(void) {
  uint8_t read_buf[READ_BYTES_LIMIT] = {0};
  memset(read_buf, 0, sizeof(read_buf));
  memset(rec_buffer_, 0, sizeof(rec_buffer_));
  uint16_t bytes_get = 0;
  while (running_) {
    while (running_ && bytes_get == 0) {
      bytes_get = Rs485_device_ptr_.read(read_buf, sizeof(read_buf));
      usleep(20000);  // read 20ms a time
    }
    if (bytes_get > 0) {
      // for(int i = 0; i < 8; i++) {
      //     printf("read_buf[%d] = %d\n", i, read_buf[i]);
      // }
      // debug, print read data from serial.
      if (bytes_get >= READ_BYTES_LIMIT) {
        printf("read bytes >= READ_BYTES_LIMIT(128), data may loss\n");
        bytes_get = READ_BYTES_LIMIT;
      }
      if (buf_write_pos_ + bytes_get >= REC_BUF_LEN) {
        printf("rec_buffer_ full !!!, data may loss\n");
        memcpy(rec_buffer_, read_buf, REC_BUF_LEN - buf_write_pos_ - 1);
        buf_write_pos_ = 0;
      } else {
        memcpy(rec_buffer_, read_buf, bytes_get);
        buf_write_pos_ += bytes_get;
      }
    }
    bytes_get = 0;
    if (buf_write_pos_ >= MIN_RESULT_LEN) {
      RecBuffParse();
    }
  }
}

void UartRs485Interface::InitDevice(void) {
  if (Rs485_device_ptr_.isOpen()) {
    std::cout << " serial /dev/ttyUSB0 init ok !\n " << std::endl;
    serial_rec_pthread_ = std::thread(&UartRs485Interface::ThreadReadRec, this);
  } else {
    std::cout << " serial /dev/ttyUSB0 init failed !\n " << std::endl;
  }
}
uint8_t UartRs485Interface::WriteDevice(uint8_t *data, uint8_t size) {
  return Rs485_device_ptr_.write(data, size);
}

uint8_t UartRs485Interface::ReadDevice(uint8_t *buf, uint8_t size) {
  return Rs485_device_ptr_.read(buf, size);
}
