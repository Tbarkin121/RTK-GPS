# Copyright 2016 Open Source Robotics Foundation, Inc.
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

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from ubxmsg.msg import SVIN, PVT
import serial
import time

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription_svin = self.create_subscription(
            SVIN,
            'svin_msg',
            self.listener_callback1,
            10)
        self.subscription_pvt = self.create_subscription(
            PVT,
            'pvt_msg',
            self.listener_callback2,
            10)


        self.subscription_svin  # prevent unused variable warning
        self.subscription_pvt  # prevent unused variable warning

    def listener_callback1(self, msg):
        self.get_logger().info('I heard a message')
        print(msg)

    def listener_callback2(self, msg):
        self.get_logger().info('I heard a message')
        print(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
