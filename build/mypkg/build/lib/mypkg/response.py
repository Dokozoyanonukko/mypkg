#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Dokozoyanonukko
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from system_msgs.msg import Response

TAKEN = 1
NO_RESPONSE = 0

class ResponseNode(Node):
    def __init__(self):
        super().__init__('response_node')

        self.publisher = self.create_publisher(
            Response,
            'response',
            10
        )

        self.count = 0

        # 5秒おきに反応を送る
        self.timer = self.create_timer(5.0, self.timer_callback)

    def timer_callback(self):
        msg = Response()

        # 3回に1回だけ TAKEN にする
        if self.count % 4 == 3:
            msg.response = TAKEN
        else:
            msg.response = NO_RESPONSE

        self.publisher.publish(msg)
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ResponseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
