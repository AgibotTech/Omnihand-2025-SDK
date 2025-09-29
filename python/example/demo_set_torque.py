# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from omnihand_2025 import AgibotHandO10, EFinger, EControlMode, EHandType
import time

def main():
    hand = AgibotHandO10.create_hand(cfg_path="./conf/hardware_conf.yaml")

    hand.show_data_details(True)

    # for i in range(10):
    #     print("Control mode of joint motor {}: ".format(i+1), hand.get_control_mode(i+1))

    
    
    all_control_modes = hand.get_all_control_modes()
    print("All control modes: ", all_control_modes)


if __name__ == "__main__":
    main()