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
from pyubx2 import UBXReader

class MinimalPublisher(Node):
    

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_svin = self.create_publisher(SVIN, 'svin_msg', 10)
        self.publisher_pvt = self.create_publisher(PVT, 'pvt_msg', 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.ser = serial.Serial("/dev/ttyACM0", 57600, timeout=3)
        self.ubr = UBXReader(self.ser, protfilter=7) # UBS + NMEA?
        self.ser.reset_input_buffer()

    def timer_callback(self):
        msg_svin = SVIN()
        msg_pvt = PVT()

        inWaiting_old = 0

        rec_svin = False
        rec_pvt = False
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
                    msg_svin.time = parsed_data.iTOW
                    msg_svin.dur = parsed_data.dur
                    msg_svin.mean_x = float(parsed_data.meanX)
                    msg_svin.mean_y = float(parsed_data.meanY)
                    msg_svin.mean_z = float(parsed_data.meanZ)
                    msg_svin.mean_xhp = float(parsed_data.meanXHP)
                    msg_svin.mean_yhp = float(parsed_data.meanYHP)
                    msg_svin.mean_zhp = float(parsed_data.meanZHP)
                    msg_svin.mean_acc = float(parsed_data.meanAcc)
                    msg_svin.obs_time = parsed_data.obs
                    msg_svin.valid = bool(parsed_data.valid)
                    msg_svin.active = bool(parsed_data.active)
                    rec_svin = True
                
                if(parsed_data.identity == 'NAV-PVT'):
                    msg_pvt.time = parsed_data.iTOW
                    msg_pvt.year = parsed_data.year
                    msg_pvt.month = parsed_data.month
                    msg_pvt.day = parsed_data.day
                    msg_pvt.hour = parsed_data.hour
                    msg_pvt.min = parsed_data.min
                    msg_pvt.second = parsed_data.second
                    msg_pvt.valid_data = bool(parsed_data.validDate)
                    msg_pvt.valid_time = bool(parsed_data.validTime)
                    msg_pvt.fully_resolved = bool(parsed_data.fullyResolved)
                    msg_pvt.valid_mag = bool(parsed_data.validMag)
                    msg_pvt.t_acc = parsed_data.tAcc
                    msg_pvt.nano = parsed_data.nano
                    msg_pvt.fix_type = parsed_data.fixTime
                    msg_pvt.gnss_fix_ok = bool(parsed_data.gnssFixOk)
                    msg_pvt.dif_soln = bool(parsed_data.difSoln)
                    msg_pvt.psm_state = parsed_data.psmState
                    msg_pvt.head_veh_valid = bool(parsed_data.headVehValid)
                    msg_pvt.carr_soln = bool(parsed_data.carrSoln)
                    msg_pvt.comfirmed_avai = bool(parsed_data.confirmedAvai)
                    msg_pvt.confirmed_data = bool(parsed_data.confirmedDate)
                    msg_pvt.confirmed_time = bool(parsed_data.confirmedTime)
                    msg_pvt.num_sv = parsed_data.numSV
                    msg_pvt.lon = float(parsed_data.lon)
                    msg_pvt.lat = float(parsed_data.lat)
                    msg_pvt.height = float(parsed_data.height)
                    msg_pvt.h_msl = parsed_data.hMSL
                    msg_pvt.h_acc = parsed_data.hAcc
                    msg_pvt.v_acc = parsed_data.vAcc
                    msg_pvt.vel_n = float(parsed_data.velN)
                    msg_pvt.vel_e = float(parsed_data.velE)
                    msg_pvt.vel_d = float(parsed_data.velD)
                    msg_pvt.ground_speed = float(parsed_data.gSpeed)
                    msg_pvt.head_mot = float(parsed_data.headMot)
                    msg_pvt.s_acc = parsed_data.sAcc
                    msg_pvt.head_acc = float(parsed_data.headAcc)
                    msg_pvt.p_dop = float(parsed_data.pDOP)
                    msg_pvt.invalid_l1h = bool(parsed_data.invalidL1h)
                    msg_pvt.last_correction_age = parsed_data.lastCorrectionAge
                    msg_pvt.head_veh = float(parsed_data.headVeh)
                    msg_pvt.mag_dec = float(parsed_data.magDec)
                    msg_pvt.mag_acc = float(parsed_data.magAcc)
                    rec_pvt = True

                # print(parsed_data)
                # print('\n')
                if(self.ser.inWaiting() == inWaiting_old):
                    # print('Not Reading Anything')
                    break
                inWaiting_old = self.ser.inWaiting()


            print(rec_svin)
            if(rec_svin):    
                self.publisher_svin.publish(msg_svin)
                print(msg_svin)

            if(rec_pvt):    
                self.publisher_pvt.publish(msg_pvt)
                print(msg_pvt)
            
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
