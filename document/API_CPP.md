# OmniHand 灵动款 2025 SDK C++ API

## 枚举类型

### EFinger (手指枚举)

```cpp
enum class EFinger : unsigned char {
    eThumb = 0x01,    // 拇指
    eIndex = 0x02,    // 食指
    eMiddle = 0x03,   // 中指
    eRing = 0x04,     // 无名指
    eLittle = 0x05,   // 小指
    ePalm = 0x06,     // 手心
    eDorsum = 0x07,   // 手背
    eUnknown = 0xff   // 未知
};
```

### EControlMode (控制模式枚举)

```cpp
enum class EControlMode : unsigned char {
  ePosi = 0,            // 位置模式
  eServo = 1,           // 伺服模式
  eVelo = 2,            // 速度模式
  eTorque = 3,          // 力控模式
  ePosiTorque = 4,      // 位置力控模式（暂不支持）
  eVeloTorque = 5,      // 速度力控模式（暂不支持）
  ePosiVeloTorque = 6,  // 位置速度力控模式（暂不支持）
  eUnknown = 10         // 未知模式
};
```

### EMsgType (消息类型枚举)

```cpp
enum class EMsgType : unsigned char {
    eVendorInfo = 0x01,           // 厂家信息
    eDeviceInfo = 0x02,           // 设备信息
    eCurrentThreshold = 0x03,     // 电流阈值
    eTactileSensor = 0x05,          // 触觉传感器
    eCtrlMode = 0x10,             // 控制模式
    eTorqueCtrl = 0x11,           // 力矩控制
    eVeloCtrl = 0x10,             // 速度控制
    ePosiCtrl = 0x13,             // 位置控制
    eMixCtrl = 0x14,              // 混合控制
    eErrorReport = 0x20,          // 错误报告
    eTemperatureReport = 0x21,    // 温度报告
    eCurrentReport = 0x22,        // 电流报告
};
```

## 数据结构

### VendorInfo(厂商信息)

```cpp
struct VendorInfo {
    std::string product_model;    // 产品型号
    std::string product_seq_num;  // 产品序列号
    Version hardware_version;     // 硬件版本
    Version software_version;     // 软件版本
    int16_t voltage;             // 供电电压(mV)
    uint8_t dof;                 // 主动自由度

    std::string toString() const;
};
```

### DeviceInfo(设备信息)

```cpp
struct AGIBOT_EXPORT CommuParams {
  unsigned char bitrate_;
  unsigned char sample_point_;
  unsigned char dbitrate_;
  unsigned char dsample_point_;
};

struct AGIBOT_EXPORT DeviceInfo {
  unsigned char deviceId;   // 设备ID
  CommuParams commuParams;  // 通信参数
};

```

### JointMotorErrorReport (关节电机错误报告)

```cpp
struct JointMotorErrorReport {
    unsigned char stalled_ : 1;      // 堵转标志
    unsigned char overheat_ : 1;     // 过热标志
    unsigned char over_current_ : 1; // 过流标志
    unsigned char motor_except_ : 1; // 电机异常
    unsigned char commu_except_ : 1; // 通信异常
    unsigned char res1_ : 3;         // 保留位
    unsigned char res2_;             // 保留字节
};
```

### MixCtrl (混合控制结构)

```cpp
struct MixCtrl {
    unsigned char joint_index_ : 5;      // 关节索引 (1-10)
    unsigned char ctrl_mode_ : 3;        // 控制模式
    std::optional<short> tgt_posi_;      // 目标位置
    std::optional<short> tgt_velo_;      // 目标速度
    std::optional<short> tgt_torque_;    // 目标力矩
};
```

### CanId (CAN 报文 ID 结构)

```cpp
struct CanId {
    unsigned char device_id_ : 7;    // 设备ID
    unsigned char rw_flag_ : 1;      // 读写标志
    unsigned char product_id_ : 7;   // 产品ID
    unsigned char res1 : 1;          // 保留位
    unsigned char msg_type_;         // 消息类型
    unsigned char msg_id_;           // 消息ID
};
```

### Version (版本信息结构)

```cpp
struct Version {
    unsigned char major_;    // 主版本号
    unsigned char minor_;    // 次版本号
    unsigned char patch_;    // 补丁版本号
    unsigned char res_;      // 保留字节
};
```

### CommuParams (通信参数结构)

```cpp
struct CommuParams {
    unsigned char bitrate_;      // 波特率
    unsigned char sample_point_; // 采样点
    unsigned char dbitrate_;     // 数据波特率
    unsigned char dsample_point_; // 数据采样点
};
```

## AgibotHandO10 类及其函数接口

### 创建灵巧手实例

```cpp
/**
    * @brief 创建灵巧手实例
    * @param device_id 设备ID，默认为1
    * @param hand_type 手类型，默认为左手
    * @param cfg_path 配置文件路径（yaml 格式）
    * @return 灵巧手对象指针
    */
static std::shared_ptr<AgibotHandO10> CreateHand(
    unsigned char device_id = 1,
    EHandType hand_type = EHandType::eLeft,
    const std::string& cfg_path = "");
```

### 构造函数

```cpp
/**
 * @brief 构造函数
 * @param device_id 设备ID，默认为1
 */
explicit AgibotHandO10();
```

### 设备信息相关

```cpp
/**
 * @brief 获取厂家信息
 * @return 厂家信息长字符串，包含产品型号、序列号、硬件版本、软件版本等信息
 */
std::string GetVendorInfo();

/**
 * @brief 获取设备信息
 * @return 设备信息长字符串，包含设备的运行状态信息
 * @note 串口暂不支持该接口
 */
std::string GetDeviceInfo();

/**
 * @brief 设置设备ID
 * @param device_id 设备ID
 * @note 串口暂不支持该接口
 */
void SetDeviceId(unsigned char device_id);
```

### 电机位置控制

```cpp
/**
 * @brief 设置单个关节电机位置
 * @param joint_motor_index 关节电机索引 (1-10)
 * @param posi 电机位置，范围：0~2000
 */
void SetJointMotorPosi(unsigned char joint_motor_index, short posi);

/**
 * @brief 获取单个关节电机位置
 * @param joint_motor_index 关节电机索引 (1-10)
 * @return 当前位置值
 */
short GetJointMotorPosi(unsigned char joint_motor_index);

/**
 * @brief 批量设置所有关节电机位置
 * @param vec_posi 所有关节的目标位置向量，长度必须为10
 * @note 注意要提供完整的10个关节电机的位置数据
 */
void SetAllJointMotorPosi(std::vector<short> vec_posi);

/**
 * @brief 批量获取所有关节电机位置
 * @return 所有关节的当前位置向量，长度为10
 */
std::vector<short> GetAllJointMotorPosi();
```

### 关节角位置控制

#### 关节角输出/输入顺序（右手）

| 序号 | 关节名称           | 最小角度(rad)        | 最大角度(rad)       | 最小角度(°) | 最大角度(°) | 速度限制(rad/s) |
| ---- | ------------------ | -------------------- | ------------------- | ----------- | ----------- | --------------- |
| 1    | R_thumb_roll_joint | -0.17453292519943295 | 0.8726646259971648  | -10         | 50          | 0.164           |
| 2    | R_thumb_abad_joint | -1.7453292519943295  | 0                   | -100        | 0           | 0.164           |
| 3    | R_thumb_mcp_joint  | 0                    | 0.8552113334772214  | 0           | 49          | 0.308           |
| 4    | R_index_abad_joint | -0.20943951023931953 | 0                   | -12         | 0           | 0.164           |
| 5    | R_index_pip_joint  | 0                    | 1.5707963267948966  | 0           | 90          | 0.308           |
| 6    | R_middle_pip_joint | 0                    | 1.5707963267948966  | 0           | 90          | 0.308           |
| 7    | R_ring_abad_joint  | 0                    | 0.17453292519943295 | 0           | 10          | 0.164           |
| 8    | R_ring_pip_joint   | 0                    | 1.5707963267948966  | 0           | 90          | 0.308           |
| 9    | R_pinky_abad_joint | 0                    | 0.17453292519943295 | 0           | 10          | 0.164           |
| 10   | R_pinky_pip_joint  | 0                    | 1.5707963267948966  | 0           | 90          | 0.308           |

#### 关节角输出/输入顺序（左手）

| 序号 | 关节名称           | 最小角度(rad)        | 最大角度(rad)       | 最小角度(°) | 最大角度(°) | 速度限制(rad/s) |
| ---- | ------------------ | -------------------- | ------------------- | ----------- | ----------- | --------------- |
| 1    | L_thumb_roll_joint | -0.8726646259971648  | 0.17453292519943295 | -50         | 10          | 0.164           |
| 2    | L_thumb_abad_joint | 0                    | 1.7453292519943295  | 0           | 100         | 0.164           |
| 3    | L_thumb_mcp_joint  | -0.8552113334772214  | 0                   | -49         | 0           | 0.308           |
| 4    | L_index_abad_joint | 0                    | 0.20943951023931953 | 0           | 12          | 0.164           |
| 5    | L_index_pip_joint  | 0                    | 1.5707963267948966  | 0           | 90          | 0.308           |
| 6    | L_middle_pip_joint | 0                    | 1.5707963267948966  | 0           | 90          | 0.308           |
| 7    | L_ring_abad_joint  | -0.17453292519943295 | 0                   | -10         | 0           | 0.164           |
| 8    | L_ring_pip_joint   | 0                    | 1.5707963267948966  | 0           | 90          | 0.308           |
| 9    | L_pinky_abad_joint | -0.17453292519943295 | 0                   | -10         | 0           | 0.164           |
| 10   | L_pinky_pip_joint  | 0                    | 1.5707963267948966  | 0           | 90          | 0.308           |

```cpp
/**
    * @brief 设置所有主动关节角度
    * @param angles 关节角度向量（单位：弧度），长度必须为10
    * @note 具体输出顺序和限位请参考 assets 模型文件
    */
void SetAllActiveJointAngles(const std::vector<double>& angles);

/**
    * @brief 获取所有主动关节角度
    * @return 关节角度向量（单位：弧度），长度为10
    * @note 具体输出顺序和限位请参考 assets 模型文件
    */
std::vector<double> GetAllActiveJointAngles() const;

/**
    * @brief 获取所有关节角度（包括主动和被动）
    * @return 关节角度向量（单位：弧度）
    * @note 具体输出顺序和限位请参考 assets 模型文件
    */
std::vector<double> GetAllJointAngles() const;
```

### 速度控制

```cpp
/**
 * @brief 设置单个关节电机速度
 * @param joint_motor_index 关节电机索引 (1-10)
 * @param velo 目标速度值
 * @note 串口暂不支持该接口
 */
void SetJointMotorVelo(unsigned char joint_motor_index, short velo);

/**
 * @brief 获取单个关节电机速度
 * @param joint_motor_index 关节电机索引 (1-10)
 * @return 当前速度值
 * @note 串口暂不支持该接口
 */
short GetJointMotorVelo(unsigned char joint_motor_index);

/**
 * @brief 批量设置所有关节电机速度
 * @param vec_velo 所有关节的目标速度向量，长度必须为10
 */
void SetAllJointMotorVelo(std::vector<short> vec_velo);

/**
 * @brief 批量获取所有关节电机速度
 * @return 所有关节的当前速度向量，长度为10
 */
std::vector<short> GetAllJointMotorVelo();
```

### 传感器数据

```cpp
/**
 * @brief 获取指定手指的触觉传感器数据
 * @param eFinger 手指枚举值
 * @return 对应手指的触觉传感器数据列表，如果是手指传感器则长度为16， 如果是手掌/手心长度为25
 */
std::vector<uint8_t> GetTactileSensorData(EFinger eFinger);
```

### 控制模式

```cpp
/**
 * @brief 设置单个关节电机控制模式
 * @param joint_motor_index 关节电机索引 (1-10)
 * @param mode 控制模式枚举值
 */
void SetControlMode(unsigned char joint_motor_index, EControlMode mode);

/**
 * @brief 获取单个关节电机控制模式
 * @param joint_motor_index 关节电机索引 (1-10)
 * @return 当前控制模式
 * @note 串口暂不支持该接口
 */
EControlMode GetControlMode(unsigned char joint_motor_index);

/**
 * @brief 批量设置所有关节电机控制模式
 * @param vec_ctrl_mode 控制模式向量，长度必须为10
 * @note 串口暂不支持该接口
 */
void SetAllControlMode(std::vector<unsigned char> vec_ctrl_mode);

/**
 * @brief 批量获取所有关节电机控制模式
 * @return 控制模式向量，长度为10
 * @note 串口暂不支持该接口
 */
std::vector<unsigned char> GetAllControlMode();
```

### 电流阈值控制

```cpp
/**
 * @brief 设置单个关节电机电流阈值
 * @param joint_motor_index 关节电机索引 (1-10)
 * @param current_threshold 电流阈值
 * @note 串口暂不支持该接口
 */
void SetCurrentThreshold(unsigned char joint_motor_index, short current_threshold);

/**
 * @brief 获取单个关节电机电流阈值
 * @param joint_motor_index 关节电机索引 (1-10)
 * @return 当前电流阈值
 * @note 串口暂不支持该接口
 */
short GetCurrentThreshold(unsigned char joint_motor_index);

/**
 * @brief 批量设置所有关节电机电流阈值
 * @param vec_current_threshold 电流阈值向量，长度必须为10
 * @note 串口暂不支持该接口
 */
void SetAllCurrentThreshold(std::vector<short> vec_current_threshold);

/**
 * @brief 批量获取所有关节电机电流阈值
 * @return 电流阈值向量，长度为10
 * @note 串口暂不支持该接口
 */
std::vector<short> GetAllCurrentThreshold();
```

### 混合控制

```cpp
/**
 * @brief 混合控制关节电机
 * @param vec_mix_ctrl 混合控制参数向量
 * @note 串口暂不支持该接口
 */
void MixCtrlJointMotor(std::vector<MixCtrl> vec_mix_ctrl);
```

### 错误处理

```cpp
/**
 * @brief 获取单个关节电机错误报告
 * @param joint_motor_index 关节电机索引 (1-10)
 * @return 错误报告结构
 */
JointMotorErrorReport GetErrorReport(unsigned char joint_motor_index);

/**
 * @brief 获取所有关节电机错误报告
 * @return 错误报告向量，长度为10
 */
std::vector<JointMotorErrorReport> GetAllErrorReport();

```

### 温度监控

```cpp
/**
 * @brief 获取单个关节电机温度报告
 * @note 查询前需要先设置上报周期
 * @param joint_motor_index 关节电机索引 (1-10)
 * @return 当前温度值
 */
unsigned short GetTemperatureReport(unsigned char joint_motor_index);

/**
 * @brief 获取所有关节电机温度报告
 * @note 查询前需要先设置上报周期
 * @return 温度值向量，长度为10
 */
std::vector<unsigned short> GetAllTemperatureReport();

```

### 电流监控

```cpp
/**
 * @brief 获取单个关节电机电流报告
 * @note 查询前需要先设置上报周期
 * @param joint_motor_index 关节电机索引 (1-10)
 * @return 当前电流值
 */
short GetCurrentReport(unsigned char joint_motor_index);

/**
 * @brief 获取所有关节电机电流报告
 * @note 查询前需要先设置上报周期
 * @return 电流值向量，长度为10
 */
std::vector<unsigned short> GetAllCurrentReport();

```

### 调试功能

```cpp
/**
 * @brief 显示发送接收数据细节
 * @param show 是否显示数据细节
 */
void ShowDataDetails(bool show) const;
```
