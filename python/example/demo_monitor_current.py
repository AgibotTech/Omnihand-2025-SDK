# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from omnihand_2025 import AgibotHandO10, EFinger, EControlMode
import time

def main():
    hand = AgibotHandO10.create_hand()
    
    #  get currents report for finger 8
    temp = hand.get_current_report(8)
    print(f"关节8温度: {temp}°C")

    all_currents = hand.get_all_current_reports()
    print(f"所有关节电流: {all_currents}")


if __name__ == "__main__":
    main()