# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from agibot_hand import AgibotHandO10, EFinger, EControlMode
import time

def main():
    hand = AgibotHandO10()

    hand.set_joint_position(2, 200)
    time.sleep(1)

    id2_joint_posi = hand.get_joint_position(2)
    print("Joint 2 position: ", id2_joint_posi)

    init_positions = [2048,2048,4096,2048,4096,4096,2048,4096,2048,4096]
    hand.set_all_joint_positions(init_positions)
    time.sleep(1)

    real_positions = hand.get_all_joint_positions()
    print("All joint positions: ", real_positions)


if __name__ == "__main__":
    main()