# OmniHand 灵动款 2025 SDK Python API

## 枚举类型

### EFinger (手指枚举)

```python
# 可用值
class EFinger(IntEnum):
    THUMB = 1
    INDEX = 2
    MIDDLE = 3
    RING = 4
    LITTLE = 5,
    PALM = 6,
    DORSUM = 7,
    UNKNOWN = 255
```

### EControlMode (控制模式枚举)

```python
class EControlMode(IntEnum):
    POSITION = 0         # 位置模式
    SERVO = 1           # 伺服模式
    VELOCITY = 2        # 速度模式
    TORQUE = 3          # 力控模式
    POSITION_TORQUE = 4 # 位置力控模式（暂不支持）
    VELOCITY_TORQUE = 5 # 速度力控模式（暂不支持）
    POSITION_VELOCITY_TORQUE = 6 # 位置速度力控模式（暂不支持）
    UNKNOWN = 10
```

## 数据结构

### VendorInfo(厂商信息)

```python
class VendorInfo:
    product_model: str
    product_seq_num: str
    hardware_version: Version
    software_version: Version
    voltage: int
    dof: int

    def __init__(self) -> None: ...

    def __str__(self) -> str: ...
```

### DeviceInfo(设备信息)

```python
class CommuParams:
    bitrate_: int
    sample_point_: int
    dbitrate_: int
    dsample_point_: int

    def __init__(self) -> None: ...

class DeviceInfo:
    device_id: int
    commu_params: CommuParams

    def __init__(self) -> None: ...

    def __str__(self) -> str: ...

```

### JointMotorErrorReport (关节电机错误报告)

```python
class JointMotorErrorReport:
    stalled: bool          # 堵转标志
    overheat: bool         # 过热标志
    over_current: bool     # 过流标志
    motor_except: bool     # 电机异常
    commu_except: bool     # 通信异常
```

### MixCtrl (混合控制结构)

```python
class MixCtrl:
    joint_index: int               # 关节索引 (1-10)
    ctrl_mode: int                 # 控制模式
    tgt_posi: Optional[int]        # 目标位置
    tgt_velo: Optional[int]        # 目标速度
    tgt_torque: Optional[int]      # 目标力矩
```

## 核心类

### AgibotHandO10

主要的灵巧手控制类，提供所有控制接口。

```python
class AgibotHandO10:
    @staticmethod
    def create_hand(device_id: int = 1,
                   hand_type: EHandType = EHandType.LEFT,
                   cfg_path: str="") -> 'AgibotHandO10': ...
    """创建灵巧手对象
    Args:
        device_id: 设备ID，默认为1
        hand_type: 手类型，默认为左手
        cfg_path: 配置文件路径，默认为空，使用默认配置
    """

    def __init__(self):
        """初始化灵巧手对象"""
        pass

    # 设备信息相关
    def get_vendor_info(self) -> str:
        """获取厂家信息"""
        pass

    def get_device_info(self) -> str:
        """获取设备信息"""
        pass

    def set_device_id(self, device_id: int) -> None:
        """设置设备ID"""
        pass

    # 位置控制
    def set_joint_position(self, joint_motor_index: int, position: int) -> None:
        """设置单个关节电机位置"""
        pass

    def get_joint_position(self, joint_motor_index: int) -> int:
        """获取单个关节电机位置"""
        pass

    def set_all_joint_positions(self, positions: List[int]) -> None:
        """批量设置所有关节电机位置"""
        pass

    def get_all_joint_positions(self) -> List[int]:
        """批量获取所有关节电机位置"""
        pass

    # 速度控制
    def set_joint_velocity(self, joint_motor_index: int, velocity: int) -> None:
        """设置单个关节电机速度"""
        pass

    def get_joint_velocity(self, joint_motor_index: int) -> int:
        """获取单个关节电机速度"""
        pass

    def set_all_joint_velocities(self, velocities: List[int]) -> None:
        """批量设置所有关节电机速度"""
        pass

    def get_all_joint_velocities(self) -> List[int]:
        """批量获取所有关节电机速度"""
        pass

    # 传感器数据
    def get_touch_sensor_data(self, finger: Finger) -> List[int]:
        """获取指定手指的触觉传感器数据阵列（一维向量）"""
        pass

    # 控制模式
    def set_control_mode(self, joint_motor_index: int, mode: ControlMode) -> None:
        """设置单个关节电机控制模式"""
        pass

    def get_control_mode(self, joint_motor_index: int) -> ControlMode:
        """获取单个关节电机控制模式"""
        pass

    def set_all_control_modes(self, modes: List[int]) -> None:
        """批量设置所有关节电机控制模式"""
        pass

    def get_all_control_modes(self) -> List[int]:
        """批量获取所有关节电机控制模式"""
        pass

    # 电流阈值控制
    def set_current_threshold(self, joint_motor_index: int, current_threshold: int) -> None:
        """设置单个关节电机电流阈值"""
        pass

    def get_current_threshold(self, joint_motor_index: int) -> int:
        """获取单个关节电机电流阈值"""
        pass

    def set_all_current_thresholds(self, current_thresholds: List[int]) -> None:
        """批量设置所有关节电机电流阈值"""
        pass

    def get_all_current_thresholds(self) -> List[int]:
        """批量获取所有关节电机电流阈值"""
        pass

    # 混合控制
    def mix_ctrl_joint_motor(self, mix_ctrls: List[MixCtrl]) -> None:
        """混合控制关节电机"""
        pass

    # 错误处理
    def get_error_report(self, joint_motor_index: int) -> JointMotorErrorReport:
        """获取单个关节电机错误报告"""
        pass

    def get_all_error_reports(self) -> List[JointMotorErrorReport]:
        """获取所有关节电机错误报告"""
        pass

    # 温度监控
    def get_temperature_report(self, joint_motor_index: int) -> int:
        """获取单个关节电机温度报告"""
        pass

    def get_all_temperature_reports(self) -> List[int]:
        """获取所有关节电机温度报告"""
        pass


    # 电流监控
    def get_current_report(self, joint_motor_index: int) -> int:
        """获取单个关节电机电流报告"""
        pass

    def get_all_current_reports(self) -> List[int]:
        """获取所有关节电机电流报告"""
        pass


    # 调试功能
    def show_data_details(self, show: bool) -> None:
        """显示发送接收数据细节"""
        pass
```

## 详细 API 说明

### 设备信息相关

```python
def get_vendor_info(self) -> str:
    """获取厂家信息

    Returns:
        str: 厂家信息长字符串，包含产品型号、序列号、硬件版本、软件版本等信息

    """

def get_device_info(self) -> str:
    """获取设备信息

    Returns:
        str: 设备信息长字符串，包含设备的运行状态信息

    Note:
        串口暂不支持该接口
    """

def set_device_id(self, device_id: int) -> None:
    """设置设备ID

    Args:
        device_id: 设备ID

    Note:
        串口暂不支持该接口
    """
```

### 电机位置控制

```python
def set_joint_position(self, joint_motor_index: int, position: int) -> None:
    """设置单个关节电机位置

    Args:
        joint_motor_index: 关节电机索引 (1-10)
        position: 电机位置，范围：0~2000
    """

def get_joint_position(self, joint_motor_index: int) -> int:
    """获取单个关节电机位置

    Args:
        joint_motor_index: 关节电机索引 (1-10)

    Returns:
        int: 当前位置值
    """

def set_all_joint_positions(self, positions: List[int]) -> None:
    """批量设置所有关节电机位置

    Args:
        positions: 所有关节的目标位置列表，长度必须为10

    Note:
        需要提供完整的10个关节电机的位置数据
    """

def get_all_joint_positions(self) -> List[int]:
    """批量获取所有关节电机位置

    Returns:
        List[int]: 所有关节的当前位置列表，长度为10
    """
```

### 关节角控制

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

```python
def set_all_active_joint_angles(self, vec_angle: List[float]) -> None: ...
    """设置所有主动关节角（单位：弧度）

    Args:
        vec_angle，所有主动关节目标关节角列表，长度必须为10

    Note:
        具体输出顺序和限位请参考 assets 模型文件
    """

def get_all_active_joint_angles(self) -> List[float]: ...
    """获取所有主动关节角（单位：弧度）

    Returns:
        List[float]: 所有主动关节当前关节角列表，长度为10

    Note:
        具体输出顺序和限位请参考 assets 模型文件
    """

def get_all_joint_angles(self) -> List[float]: ...
    """获取所有主动和被动关节角（单位：弧度）

    Returns:
        List[float]: 所有主动和被动关节当前关节角列表，长度为10

    Note:
        具体输出顺序和限位请参考 assets 模型文件
    """
```

### 速度控制

```python
def set_joint_velocity(self, joint_motor_index: int, velocity: int) -> None:
    """设置单个关节电机速度

    Args:
        joint_motor_index: 关节电机索引 (1-10)
        velocity: 目标速度值

    Note:
        串口暂不支持该接口
    """

def get_joint_velocity(self, joint_motor_index: int) -> int:
    """获取单个关节电机速度

    Args:
        joint_motor_index: 关节电机索引 (1-10)

    Returns:
        int: 当前速度值

    Note:
        串口暂不支持该接口
    """

def set_all_joint_velocities(self, velocities: List[int]) -> None:
    """批量设置所有关节电机速度

    Args:
        velocities: 所有关节的目标速度列表，长度必须为10
    """

def get_all_joint_velocities(self) -> List[int]:
    """批量获取所有关节电机速度

    Returns:
        List[int]: 所有关节的当前速度列表，长度为10
    """
```

### 传感器数据

```python
def get_touch_sensor_data(self, finger: Finger) -> List[int]:
    """获取指定手指的触觉传感器数据

    Args:
        finger: 手指枚举值，可选值：
                Finger.THUMB,
                Finger.INDEX,
                Finger.MIDDLE,
                Finger.RING,
                Finger.LITTLE，
                Finger.PALM，
                Finger.DORSUM

    Returns:
        List[int]: 对应手指的触觉传感器数据列表，如果是手指传感器则长度为16， 如果是手掌/手心长度为25
    """
```

### 控制模式

```python
def set_control_mode(self, joint_motor_index: int, mode: ControlMode) -> None:
    """设置单个关节电机控制模式

    Args:
        joint_motor_index: 关节电机索引 (1-10)
        mode: 控制模式枚举值
    """

def get_control_mode(self, joint_motor_index: int) -> ControlMode:
    """获取单个关节电机控制模式

    Args:
        joint_motor_index: 关节电机索引 (1-10)

    Returns:
        ControlMode: 当前控制模式

    Note:
        串口暂不支持该接口
    """

def set_all_control_modes(self, modes: List[int]) -> None:
    """批量设置所有关节电机控制模式

    Args:
        modes: 控制模式列表，长度必须为10

    Note:
        串口暂不支持该接口
    """

def get_all_control_modes(self) -> List[int]:
    """批量获取所有关节电机控制模式

    Returns:
        List[int]: 控制模式列表，长度为10

    Note:
        串口暂不支持该接口
    """
```

### 电流阈值控制

```python
def set_current_threshold(self, joint_motor_index: int, current_threshold: int) -> None:
    """设置单个关节电机电流阈值

    Args:
        joint_motor_index: 关节电机索引 (1-10)
        current_threshold: 电流阈值

    Note:
        串口暂不支持该接口
    """

def get_current_threshold(self, joint_motor_index: int) -> int:
    """获取单个关节电机电流阈值

    Args:
        joint_motor_index: 关节电机索引 (1-10)

    Returns:
        int: 当前电流阈值

    Note:
        串口暂不支持该接口
    """

def set_all_current_thresholds(self, current_thresholds: List[int]) -> None:
    """批量设置所有关节电机电流阈值

    Args:
        current_thresholds: 电流阈值列表，长度必须为10

    Note:
        串口暂不支持该接口
    """

def get_all_current_thresholds(self) -> List[int]:
    """批量获取所有关节电机电流阈值

    Returns:
        List[int]: 电流阈值列表，长度为10

    Note:
        串口暂不支持该接口
    """
```

### 混合控制

```python
def mix_ctrl_joint_motor(self, mix_ctrls: List[MixCtrl]) -> None:
    """混合控制关节电机

    Args:
        mix_ctrls: 混合控制参数列表

    Note:
        串口暂不支持该接口
    """
```

### 错误处理

```python
def get_error_report(self, joint_motor_index: int) -> JointMotorErrorReport:
    """获取单个关节电机错误报告

    Args:
        joint_motor_index: 关节电机索引 (1-10)

    Returns:
        JointMotorErrorReport: 错误报告结构
    """

def get_all_error_reports(self) -> List[JointMotorErrorReport]:
    """获取所有关节电机错误报告

    Returns:
        List[JointMotorErrorReport]: 错误报告列表，长度为10
    """
```

### 温度监控

```python
def get_temperature_report(self, joint_motor_index: int) -> int:
    """获取单个关节电机温度报告

    Note:
        查询前需要先设置上报周期

    Args:
        joint_motor_index: 关节电机索引 (1-10)

    Returns:
        int: 当前温度值
    """

def get_all_temperature_reports(self) -> List[int]:
    """获取所有关节电机温度报告

    Note:
        查询前需要先设置上报周期

    Returns:
        List[int]: 温度值列表，长度为10
    """
```

### 电流监控

```python
def get_current_report(self, joint_motor_index: int) -> int:
    """获取单个关节电机电流报告

    Note:
        查询前需要先设置上报周期

    Args:
        joint_motor_index: 关节电机索引 (1-10)

    Returns:
        int: 当前电流值
    """

def get_all_current_reports(self) -> List[int]:
    """获取所有关节电机电流报告

    Note:
        查询前需要先设置上报周期

    Returns:
        List[int]: 电流值列表，长度为10
    """
```

### 调试功能

```python
def show_data_details(self, show: bool) -> None:
    """显示发送接收数据细节

    Args:
        show: 是否显示数据细节
    """
```
