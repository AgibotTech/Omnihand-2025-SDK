# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from .omnihand_2025_core import (
    AgibotHandO10,  
    JointMotorErrorReport,
    MixCtrl,
    VendorInfo,   
    Version,     
    CommuParams,   
    DeviceInfo     
)
from enum import IntEnum

class EFinger(IntEnum):
    THUMB = 0x01
    INDEX = 0x02
    MIDDLE = 0x03
    RING = 0x04
    LITTLE = 0x05
    PALM = 0x06    
    DORSUM = 0x07  
    UNKNOWN = 0xff

class EControlMode(IntEnum):
    POSITION = 0
    VELOCITY = 1
    TORQUE = 2
    POSITION_TORQUE = 3
    VELOCITY_TORQUE = 4
    POSITION_VELOCITY_TORQUE = 5
    UNKNOWN = 10

class EHandType(IntEnum):
    LEFT = 0
    RIGHT = 1
    UNKNOWN = 10

__all__ = [
    'AgibotHandO10', 
    'EFinger',
    'EHandType',
    'EControlMode',
    'JointMotorErrorReport',
    'MixCtrl',
    'VendorInfo',   
    'Version',       
    'CommuParams',   
    'DeviceInfo'     
]