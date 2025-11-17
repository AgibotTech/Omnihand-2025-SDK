#!/usr/bin/env python3
"""
@Author: huangshiheng@agibot.com
@Date: 2025-11-06
@Description: Python node to publish hand mix control commands
"""

import rclpy
from rclpy.node import Node
import sys
import os

# 动态获取项目根路径
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(os.path.dirname(script_dir))
python_packages_path = os.path.join(project_root, "build/install/local/lib/python3.10/dist-packages")

sys.path.append(python_packages_path)

from omnihand_node_msgs.msg import MixControl


class MixControlPublisher(Node):
    def __init__(self, hand_side='left'):
        """
        初始化混合控制发布节点
        
        Args:
            hand_side: 'left' 或 'right'，表示左手或右手
        """
        super().__init__(f'{hand_side}_mix_control_publisher')
        
        self.hand_side = hand_side
        
        # 创建混合控制命令发布器
        self.mix_control_publisher = self.create_publisher(
            MixControl,
            f'/agihand/omnihand/{hand_side}/mix_control_cmd',
            10
        )

        self.timer = self.create_timer(1, self.timer_callback)
        self.get_logger().info(f'{hand_side.capitalize()} Mix Control Publisher Node started')
        self.get_logger().info(f'Publishing to: /agihand/omnihand/{hand_side}/mix_control_cmd')

    def publish_mix_control(self):
        """
        发布混合控制命令
        """
        msg = MixControl()
        # 发布消息
        # self.mix_control_publisher.publish(msg)
        mix_controls_str = ', '.join([f'{t}' for t in msg.mix_controls])
        self.get_logger().info(
            f'Publish {self.hand_side} hand: [{mix_controls_str}]'
        )

    def timer_callback(self):
        """定时器回调"""

        # 发布测试命令
        self.publish_mix_control()


def main(args=None):
    rclpy.init(args=args)
    
    # 可以通过命令行参数指定左手或右手，默认为左手
    hand_side = 'left'
    if len(sys.argv) > 1:
        hand_side = sys.argv[1]
    
    mix_control_publisher = MixControlPublisher(hand_side)

    try:
        rclpy.spin(mix_control_publisher)
    except KeyboardInterrupt:
        mix_control_publisher.get_logger().info(
            f'Shutting down {hand_side.capitalize()} Mix Control Publisher Node'
        )
    except Exception as e:
        mix_control_publisher.get_logger().error(f'Error: {str(e)}')
    finally:
        mix_control_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
