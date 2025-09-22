# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from agibot_hand import AgibotHandBase, EFinger, EControlMode, EHandType
import time
from enum import Enum


def main():
    hand = AgibotHandBase.create_hand(hand_type=EHandType.LEFT)
    hand.set_all_active_joint_angles([0]*10)
    hand.set_all_active_joint_angles([0,0,0,0,0,0,0,0,0,0])

    time.sleep(2)
    
    hand.set_all_active_joint_angles([0.866191,-0.0019873,0.105139,-0.0399219,1.48,1.48,0.00676514,1.48,0.00695801,1.48])

if __name__ == "__main__":
    main()