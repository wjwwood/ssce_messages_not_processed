#!/usr/bin/env python3
# Copyright 2023 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from threading import Thread
from time import sleep

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Header


class BurstSender(Node):
    def __init__(self):
        super().__init__('burst_sender')
        self.publisher_ = self.create_publisher(Header, 'data', 10)
        timer_period = 0.5  # seconds
        self.i = 1
        self.number_of_expected_subscriptions = 1

    def wait_for_subscriptions(self):
        matched_subscriptions = self.publisher_.get_subscription_count()
        while matched_subscriptions < self.number_of_expected_subscriptions:
            print(
                f'Waiting on subscriptions... '
                f'{matched_subscriptions}/{self.number_of_expected_subscriptions}')
            sleep(0.1)
            matched_subscriptions = self.publisher_.get_subscription_count()

    def timer_callback(self):
        msg = Header()
        msg.frame_id = 'Hello World: %d' % self.i
        msg.stamp = self.get_clock().now().to_msg()
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg}')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = BurstSender()

    spin_thread = Thread(target=rclpy.spin, args=(minimal_publisher,))
    spin_thread.start()

    minimal_publisher.wait_for_subscriptions()

    for x in range(8):
        minimal_publisher.timer_callback()

    timeout = 10
    if not minimal_publisher.publisher_.wait_for_all_acked(timeout=Duration(seconds=timeout)):
        raise RuntimeError(f"messages were not sent after {timeout} seconds")

    rclpy.shutdown()
    spin_thread.join()

if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
