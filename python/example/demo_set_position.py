# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from agibot_hand import AgibotHandO12, EFinger, EControlMode
import time

def main():
    hand = AgibotHandO12()

    a = hand.get_all_joint_positions()
    print(a)

    hand.set_joint_position(2, 2000)
    time.sleep(1)

    init_positions = [199, 201, 399, 409, 4097, 4096, 2048, 4096, 2038, 4100]
    hand.set_all_joint_positions(init_positions)
    
    # hand.set_joint_position(8, 1000)
    # time.sleep(1)

    # real_position = hand.get_joint_position(8)
    # print("Joint 8 position: ", real_position)
    # time.sleep(3)

    # aim_positions = [2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 2000, 0]
    # hand.set_all_joint_positions(aim_positions)
    # time.sleep(1)

    # real_positions = hand.get_all_joint_positions()
    # print("All joint positions: ", real_positions)


if __name__ == "__main__":
    main()