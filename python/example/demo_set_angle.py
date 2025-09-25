# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from agibot_hand import AgibotHandO10, EFinger, EControlMode, EHandType
import time
from enum import Enum


def main():
    hand = AgibotHandO10.create_hand(cfg_path="./conf/hardware_conf.yaml",hand_type=EHandType.LEFT)
    hand.set_all_active_joint_angles([0,0.26,0,0,0,-0.35,0,0,0,0.37])
    print("active joint angles:", hand.get_all_active_joint_angles())

    time.sleep(2)
    print("==============")

    hand.set_all_active_joint_angles([0.866191,-0.0019873,0.105139,-0.0399219,1.48,1.48,0.00676514,1.48,0.00695801,0.37])
    print("joint angles:", hand.get_all_joint_angles())
    

if __name__ == "__main__":
    main()