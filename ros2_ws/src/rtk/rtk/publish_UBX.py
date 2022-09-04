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
from ubxmsg import SVIN

import serial
import time

class MinimalPublisher(Node):
    

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(SVIN, 'topic', 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.ser = serial.Serial("/dev/serial0", 57600)
        self.ser.reset_input_buffer()

    def timer_callback(self):
        msg = SVIN()
        inWaiting_old = 0
        if self.ser.inWaiting()>0:

            inWaiting_old = self.ser.inWaiting()	
            while (inWaiting_old != self.ser.inWaiting()):
                inWaiting_old = self.ser.inWaiting()
                time.sleep(0.1)
            
            print('FRESH DATA, GET EM WHILE ITS HOT!')
            while(self.ser.inWaiting()):
                print('data in waiting : {}'.format(ser.inWaiting()))    
                (raw_data, parsed_data) = ubr.read()
                print(type(parsed_data))
                print(parsed_data.identity)

                if(parsed_data.identity == 'NAV-SVIN'):
                    # msg.time
                    print(parsed_data.iTOW)
                    # msg.status
                    print(parsed_data.iTOW)
                    # msg.mean_position_valid
                    print(parsed_data.iTOW)
                    # msg.obs_time
                    print(parsed_data.iTOW)
                    # msg.positions_used
                    print(parsed_data.iTOW)
                    # msg.mean_ecef_x
                    print(parsed_data.meanX)
                    # msg.mean_ecef_y
                    print(parsed_data.meanY)
                    # msg.mean_ecef_z
                    print(parsed_data.meanZ)
                    # msg.mean_3d_stddev
                    print(parsed_data.meanAcc)

                print(parsed_data)
                print('\n')
                if(self.ser.inWaiting() == inWaiting_old):
                    # print('Not Reading Anything')
                    break
                inWaiting_old = self.ser.inWaiting()

            print('Data End:')
            print('\n\n\n')
            message_recieved = True
            

            data_left = self.ser.inWaiting()             #check for remaining byte
            received_data += self.ser.read(data_left)
            msg.data = received_data.hex()

            # print(type(msg.data))
            # print(len(msg.data))
            # print(msg.data)

            self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
