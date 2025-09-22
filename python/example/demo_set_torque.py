# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from agibot_hand import AgibotHandBase, EFinger, EControlMode, EHandType
import time

def main():
    hand = AgibotHandBase.create_hand()

    all_control_modes = hand.get_all_control_modes()
    print("All control modes: ", all_control_modes)


if __name__ == "__main__":
    main()