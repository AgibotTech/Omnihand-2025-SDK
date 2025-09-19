# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from agibot_hand import AgibotHandO10, EFinger, EControlMode
import time

def main():
    hand = AgibotHandO10()

    thumb_touch_data = hand.get_touch_sensor_data(EFinger.THUMB)
    print("Thumb touch data: {} g".format(sum(thumb_touch_data)))

    index_touch_data = hand.get_touch_sensor_data(EFinger.INDEX)
    print("Index touch data: {} g".format(sum(index_touch_data)))

    middle_touch_data = hand.get_touch_sensor_data(EFinger.MIDDLE)
    print("Middle touch data: {} g".format(sum(middle_touch_data)))

    ring_touch_data = hand.get_touch_sensor_data(EFinger.RING)
    print("Ring touch data: {} g".format(sum(ring_touch_data)))

    little_touch_data = hand.get_touch_sensor_data(EFinger.LITTLE)
    print("Little touch data: {} g".format(sum(little_touch_data)))

    palm_touch_data = hand.get_touch_sensor_data(EFinger.PALM)
    print("Palm touch data: {} g".format(sum(palm_touch_data)))

    dorsum_touch_data = hand.get_touch_sensor_data(EFinger.DORSUM)
    print("Dorsum touch data: {} g".format(sum(dorsum_touch_data)))


if __name__ == "__main__":
    main()