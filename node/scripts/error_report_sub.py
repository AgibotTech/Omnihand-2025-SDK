#!/usr/bin/env python3
"""
@Author: huangshiheng@agibot.com
@Date: 2025-11-06
@Description: Python node to subscribe hand motor error report feedback (1Hz)
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

from omnihand_node_msgs.msg import MotorErrorReport


class MotorErrorReportSubscriber(Node):
    def __init__(self, hand_side='left'):
        """
        初始化电机错误报告订阅节点
        
        Args:
            hand_side: 'left' 或 'right'，表示左手或右手
        """
        super().__init__(f'{hand_side}_motor_error_report_subscriber')
        
        self.hand_side = hand_side
        
        # 创建电机错误报告反馈订阅器 (1Hz)
        self.motor_error_report_fb_subscriber = self.create_subscription(
            MotorErrorReport,
            f'/agihand/omnihand/{hand_side}/motor_error_report',
            self.motor_error_report_callback,
            10
        )
        
        # 消息计数器
        self.message_count = 0
        
        # 错误计数器 - 记录每个电机的错误次数
        self.error_counts = {}
        
        # 创建统计定时器 (10秒)
        self.stats_timer = self.create_timer(10.0, self.print_statistics)
        
        # 上一次统计时的消息计数
        self.last_count = 0
        
        self.get_logger().info(f'{hand_side.capitalize()} Motor Error Report Subscriber Node started')
        self.get_logger().info(f'Subscribing to: /agihand/omnihand/{hand_side}/motor_error_report')

    def motor_error_report_callback(self, msg):
        error_reports_str = ', '.join([f'{t}' for t in msg.error_reports])
        self.get_logger().info(
            f'Received motor error report for {self.hand_side} hand: [{error_reports_str}]'
        )

    def get_error_severity(self, error_code):
        """根据错误码获取错误严重程度"""
        # 根据实际错误码定义调整
        if error_code >= 100:
            return 'CRITICAL'
        elif error_code >= 50:
            return 'WARNING'
        else:
            return 'INFO'

    def get_error_description(self, error_code):
        """根据错误码获取错误描述"""
        # 根据实际错误码定义调整
        error_descriptions = {
            0: 'No error',
            1: 'Communication timeout',
            2: 'Overcurrent protection',
            3: 'Overvoltage protection',
            4: 'Undervoltage protection',
            5: 'Overtemperature protection',
            6: 'Position limit exceeded',
            7: 'Velocity limit exceeded',
            # 添加更多错误码定义...
        }
        return error_descriptions.get(error_code, f'Unknown error code: {error_code}')
    
    def print_statistics(self):
        """打印统计信息"""
        msg_rate = (self.message_count - self.last_count) / 10.0
        self.get_logger().info(
            f'Statistics - Total messages: {self.message_count}, Rate: {msg_rate:.2f} Hz'
        )
        
        # 打印错误统计
        if self.error_counts:
            self.get_logger().info('Error counts by motor:')
            for motor, count in sorted(self.error_counts.items()):
                self.get_logger().info(f'  {motor}: {count} errors')
        
        self.last_count = self.message_count


def main(args=None):
    rclpy.init(args=args)
    
    # 可以通过命令行参数指定左手或右手，默认为左手
    hand_side = 'left'
    if len(sys.argv) > 1:
        hand_side = sys.argv[1]
    
    motor_error_report_subscriber = MotorErrorReportSubscriber(hand_side)
    
    try:
        rclpy.spin(motor_error_report_subscriber)
    except KeyboardInterrupt:
        motor_error_report_subscriber.get_logger().info(
            f'Shutting down {hand_side.capitalize()} Motor Error Report Subscriber Node'
        )
    except Exception as e:
        motor_error_report_subscriber.get_logger().error(f'Error: {str(e)}')
    finally:
        motor_error_report_subscriber.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
