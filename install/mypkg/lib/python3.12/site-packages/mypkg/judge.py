#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Dokozoyanonukko
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from system_msgs.msg import RmEvent, RmNotioncmd, Response

# 定数
OFF = 0
NORMAL = 1
URGENT = 2
EVENT_MEDICATION_TIME = 1
RESPONSE_TAKEN = 1

class JudgeNode(Node):
    def __init__(self):
        super().__init__('judge_node')

        self.event_sub = self.create_subscription(RmEvent, 'reminder_event', self.event_callback, 10)
        self.response_sub = self.create_subscription(Response, 'response', self.response_callback, 10)
        self.publisher = self.create_publisher(RmNotioncmd, 'reminder_command', 10)

        self.last_event_time = None
        self.last_command = OFF
        self.is_waiting_taken = False

        # 0.5秒おきにタイムアウトを監視
        self.watchdog_timer = self.create_timer(0.5, self.check_status)
        self.get_logger().info('Judge Node started. Watching for medication events...')

    def event_callback(self, msg):
        if msg.event == EVENT_MEDICATION_TIME:
            # 既にNORMALかURGENTなら、リセットせずに待機継続（必要に応じて上書きも可）
            if not self.is_waiting_taken:
                self.last_event_time = self.get_clock().now()
                self.is_waiting_taken = True
                self.publish_if_changed(NORMAL)
                self.get_logger().info('Status: NORMAL (Waiting for TAKEN)')

    def response_callback(self, msg):
        if msg.response == RESPONSE_TAKEN:
            self.is_waiting_taken = False
            self.last_event_time = None
            self.publish_if_changed(OFF)
            self.get_logger().info('Status: OFF (Medication Taken)')

    def check_status(self):
        if self.is_waiting_taken and self.last_event_time is not None:
            now = self.get_clock().now()
            elapsed = (now - self.last_event_time).nanoseconds / 1e9

            # 10秒経過でURGENTに昇格
            if elapsed >= 10.0 and self.last_command != URGENT:
                self.publish_if_changed(URGENT)
                self.get_logger().warn(f'Status: URGENT ({elapsed:.1f}s elapsed!)')

    def publish_if_changed(self, command):
        if command == self.last_command:
            return
        msg = RmNotioncmd()
        msg.command = command
        self.publisher.publish(msg)
        self.last_command = command

def main(args=None):
    rclpy.init(args=args)
    node = JudgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
