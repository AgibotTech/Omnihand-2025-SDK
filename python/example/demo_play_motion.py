# Copyright (c) 2025, Agibot Co., Ltd.
# OmniHand 2025 SDK is licensed under Mulan PSL v2.

from agibot_hand import AgibotHandO10, EFinger, EControlMode, EHandType
import time
from enum import Enum
import numpy as np
import time
import platform

def precise_sleep(duration):
    """
    更精确的睡眠函数
    """
    start = time.perf_counter()
    while time.perf_counter() - start < duration:
        if platform.system() == 'Linux':
            time.sleep(0)  # 在Linux上使用yield
        else:
            pass  # 在其他系统上使用忙等待

def load_and_play_npy(file_path, frequency=10):
    """
    使用更精确的时间控制
    """
    try:
        hand = AgibotHandO10()
        hand.show_data_details(True)

        data = np.load(file_path)
        period = 1.0 / frequency
        next_time = time.perf_counter() + period
        execution_times = []
        
        for i, frame in enumerate(data):
            frame_start = time.perf_counter()
            
            # 执行控制代码
            hand.set_all_active_joint_angles(data[i])
            
            execution_time = time.perf_counter() - frame_start
            execution_times.append(execution_time)
            
            # 精确等待
            current_time = time.perf_counter()
            if next_time > current_time:
                precise_sleep(next_time - current_time)
            
            next_time += period
        
        # 计算统计信息
        actual_frequency = len(data) / (time.perf_counter() - (next_time - period))
        mean_exec_time = np.mean(execution_times) * 1000
        max_exec_time = np.max(execution_times) * 1000
        
        print("\n执行统计:")
        print(f"目标频率: {frequency}Hz")
        print(f"实际频率: {actual_frequency:.2f}Hz")
        print(f"平均执行时间: {mean_exec_time:.2f}ms")
        print(f"最大执行时间: {max_exec_time:.2f}ms")
        
    except Exception as e:
        print(f"发生错误：{e}")

def main():
    file_path = "./teleop_test_o10.npy"
    load_and_play_npy(file_path, frequency=10)


if __name__ == "__main__":
    main()