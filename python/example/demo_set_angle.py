# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from omnihand_2025 import AgibotHandO10, EFinger, EControlMode, EHandType
import time
from enum import Enum


def main():
    hand = AgibotHandO10.create_hand(cfg_path="./conf/hardware_conf.yaml",hand_type=EHandType.LEFT)

    cmds = [0,0.26,0,0,0,-0.35,0,0,0,0.37]
    hand.set_all_active_joint_angles(cmds)
    print("set active joint angles:",cmds)
    time.sleep(2)
    print("get active joint angles:", hand.get_all_active_joint_angles())

    time.sleep(2)

  
    
    print("==============")
    cmds = [0.866191,-0.0019873,0.105139,-0.0399219,1.48,1.48,0.00676514,1.48,0.00695801,0.37]
    hand.set_all_active_joint_angles(cmds)
    print("set active joint angles:",cmds)
    time.sleep(2)
    print(f"get active joint angles: ",hand.get_all_active_joint_angles())
    

if __name__ == "__main__":
    main()