#!/usr/bin/python3
# SPDX-FileCopyrightText: 2025 Dokozoyanonukko
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from system_msgs.msg import RmEvent
import sys
import time

class EventNode(Node):
    def __init__(self, production_mode=False):
        super().__init__('event_node')
        self.publisher_ = self.create_publisher(RmEvent, 'reminder_event', 10)
        self.production_mode = production_mode

        # Timerの設定 (1秒おきにチェック)
        self.timer = self.create_timer(1.0, self.publish_event)

        # 二重送信防止用の変数
        self.last_elapsed = -1

        if self.production_mode:
            self.medication_hours = [8, 20]
        else:
            self.start_time = time.time()
            self.medication_hours = None

    def publish_event(self):
        now_msg = self.get_clock().now().to_msg()
        event = RmEvent()
        event.timestamp = now_msg

        should_publish = False
        if self.production_mode:
            current_hour = (now_msg.sec // 3600) % 24
            if current_hour in self.medication_hours:
                event.event = 1
                should_publish = True
        else:
            elapsed = int(time.time() - self.start_time)
            # 5秒に1回、かつ前回の秒数と違う場合のみ送信
            if elapsed % 5 == 0 and elapsed != self.last_elapsed:
                event.event = 1
                should_publish = True
                self.last_elapsed = elapsed

        if should_publish:
            self.publisher_.publish(event)
            self.get_logger().info(f'Medication Event Sent! (event=1)')

def main(args=None):
    rclpy.init(args=args)
    production_mode = '--production' in sys.argv
    node = EventNode(production_mode=production_mode)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


