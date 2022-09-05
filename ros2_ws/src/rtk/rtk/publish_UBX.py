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
from ubxmsg.msg import SVIN

import serial
import time
from pyubx2 import UBXReader

class MinimalPublisher(Node):
    

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(SVIN, 'topic', 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.ser = serial.Serial("/dev/ttyACM0", 57600, timeout=3)
        self.ubr = UBXReader(self.ser, protfilter=2) # Just UBS
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
                print('data in waiting : {}'.format(self.ser.inWaiting()))    
                (raw_data, parsed_data) = self.ubr.read()
                # print(type(parsed_data))
                # print(parsed_data.identity)

                if(parsed_data.identity == 'NAV-SVIN'):
                    msg.time = parsed_data.iTOW
                    msg.dur = parsed_data.dur
                    msg.mean_x = float(parsed_data.meanX)
                    msg.mean_y = float(parsed_data.meanY)
                    msg.mean_z = float(parsed_data.meanZ)
                    msg.mean_xhp = float(parsed_data.meanXHP)
                    msg.mean_yhp = float(parsed_data.meanYHP)
                    msg.mean_zhp = float(parsed_data.meanZHP)
                    msg.mean_acc = float(parsed_data.meanAcc)
                    msg.obs_time = parsed_data.obs
                    msg.valid = bool(parsed_data.valid)
                    msg.active = bool(parsed_data.active)

                print(parsed_data)
                print('\n')
                if(self.ser.inWaiting() == inWaiting_old):
                    # print('Not Reading Anything')
                    break
                inWaiting_old = self.ser.inWaiting()
            
            self.publisher_.publish(msg)
            print(msg)
            print('Data End:')
            print('\n\n\n')


            
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
