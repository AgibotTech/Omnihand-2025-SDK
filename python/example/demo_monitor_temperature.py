# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from omnihand_2025 import AgibotHandO10, EFinger, EControlMode
import time

def main():
    hand = AgibotHandO10.create_hand()
        
    #  get temperature report for finger 8
    temp = hand.get_temperature_report(8)
    print(f"关节8温度: {temp}°C")

    # get temperature report for all fingers
    all_temps = hand.get_all_temperature_reports()
    print(f"所有关节温度: {all_temps}")



if __name__ == "__main__":
    main()