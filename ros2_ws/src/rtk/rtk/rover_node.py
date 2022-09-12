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

import time
from math import trunc
import serial
from pyubx2 import UBXMessage
from pyubx2 import UBXReader

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from action_base.action import Base

from std_msgs.msg import String
from ubxmsg.msg import SVIN, PVT

from rtk.ublox_functions import Ublox

class Rover(Node):
    def __init__(self):
        super().__init__('rover')
        self.uf = Ublox()

        self.PORT = "/dev/ttyACM0"
        self.BAUD = 57600
        self.TIMEOUT = 5
        self.stream = serial.Serial(self.PORT, self.BAUD, self.TIMEOUT)
        self.ubr = UBXReader(self.stream, protfilter=7) # 7 = UBS + NMEA + RTCM3
        self.stream.reset_input_buffer()

        # This is the port that the RTCM3 data will be emitted from
        self.PORT_TYPE = "UART2"  # choose from "USB", "UART1", "UART2"

        self.TMODE = self.uf.TMODE_SVIN  # "TMODE_SVIN" or 1 = Survey-In, "TMODE_FIXED" or 2 = Fixed
        self.ACC_LIMIT = 5000  # accuracy in mm
        # only used if TMODE = 1 ...
        self.SVIN_MIN_DUR = 90  # seconds

        # only used if TMODE = 2 ... (Not Set Correctly Yet... These were the example defaults)
        self.ARP_LAT = 37.012345678
        self.ARP_LON = -115.012345678
        self.ARP_HEIGHT = 137000  # cm
        
        self.publisher_svin = self.create_publisher(SVIN, 'svin_msg', 10)
        self.publisher_pvt = self.create_publisher(PVT, 'pvt_msg', 10)
        
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self._action_server = ActionServer(
            self,
            Base,
            'base',
            self.execute_callback)
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing Action...')

        feedback_msg = Base.Feedback()
        feedback_msg.progress = False

        if(goal_handle.request.action_id[0] == 1):
            feedback_msg.progress = True
            self.new_survay_in()
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.progress))

        if(goal_handle.request.action_id[0] == 2):
            self.ACC_LIMIT = goal_handle.request.action_id[1]
            feedback_msg.progress = True
            self.get_logger().info('New ACC LIMIT: {0}'.format(self.ACC_LIMIT))


        goal_handle.succeed()
        result = Base.Result()
        result.success = True
        return result
        
    def new_survay_in(self):
        msg = self.uf.config_rtcm(self.PORT_TYPE)
        self.uf.send_msg(self.stream, msg)

        msg = self.uf.config_rover(self.PORT_TYPE, self.ACC_LIMIT, self.SVIN_MIN_DUR)
        self.uf.send_msg(self.stream, msg)
        time.sleep(1)
        msg = self.uf.config_svin(self.PORT_TYPE, self.ACC_LIMIT, self.SVIN_MIN_DUR)
        self.uf.send_msg(self.stream, msg)

    def timer_callback(self):
        msg_svin = SVIN()
        msg_pvt = PVT()

        inWaiting_old = 0

        rec_svin = False
        rec_pvt = False
        if self.stream.inWaiting()>0:

            inWaiting_old = self.stream.inWaiting()	
            while (inWaiting_old != self.stream.inWaiting()):
                inWaiting_old = self.stream.inWaiting()
                time.sleep(0.1)
            
            print('FRESH DATA, GET EM WHILE ITS HOT!')
            while(self.stream.inWaiting()):
                # print('data in waiting : {}'.format(self.stream.inWaiting()))    
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
                    # print(type(parsed_data.nano))
                    msg_pvt.nano = parsed_data.nano
                    msg_pvt.fix_type = parsed_data.fixType
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
                    msg_pvt.invalid_llh = bool(parsed_data.invalidLlh)
                    msg_pvt.last_correction_age = parsed_data.lastCorrectionAge
                    msg_pvt.head_veh = float(parsed_data.headVeh)
                    msg_pvt.mag_dec = float(parsed_data.magDec)
                    msg_pvt.mag_acc = float(parsed_data.magAcc)
                    rec_pvt = True

                # print(parsed_data)
                # print('\n')
                if(self.stream.inWaiting() == inWaiting_old):
                    # print('Not Reading Anything')
                    break
                inWaiting_old = self.stream.inWaiting()


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

    rover_node = Rover()

    rclpy.spin(rover_node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rover_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
