# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from omnihand_2025 import AgibotHandO10, EFinger, EControlMode,EHandType
import time

def main():
    hand = AgibotHandO10.create_hand()
    vendor_info = hand.get_vendor_info()
    print("Vendor Info:")
    print( vendor_info)

    device_info = hand.get_device_info() 
    print("Device Info:")
    print( device_info)


if __name__ == "__main__":
    main()