#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Dokozoyanonukko
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node

from system_msgs.msg import RmNotioncmd, RmStatus


class StatusNode(Node):
    def __init__(self):
        super().__init__('status_node')

        self.subscription = self.create_subscription(
            RmNotioncmd,
            'reminder_command',
            self.command_callback,
            10
        )

        self.publisher = self.create_publisher(
            RmStatus,
            'reminder_status',
            10
        )

        # 最後に出したステータスを記憶
        self.last_status = None

    def command_callback(self, msg):
        # 同じステータスなら何もしない
        if msg.command == self.last_status:
            return

        status = RmStatus()
        status.status = msg.command
        self.publisher.publish(status)

        # 状態を更新
        self.last_status = msg.command

def main(args=None):
    rclpy.init(args=args)
    node = StatusNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
