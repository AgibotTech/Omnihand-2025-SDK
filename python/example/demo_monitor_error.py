# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from agibot_hand import AgibotHandO10, EFinger, EControlMode
import time

def main():
    hand = AgibotHandO10.create_hand(cfg_path="./conf/hardware_conf.yaml")
        
    # set error  report period for all fingers
    periods = [500] * 10
    hand.set_all_error_report_periods(periods)

    time.sleep(1)
    
    #  get error report for finger 8
    error = hand.get_error_report(8)
    print(f"joint  8 error info  :",error.over_current)

    # get error report for all fingers
    all_errors = hand.get_all_error_reports()
    for i, error in enumerate(all_errors):
        print(f"joint {i} error info: {error.stalled}")


if __name__ == "__main__":
    main()