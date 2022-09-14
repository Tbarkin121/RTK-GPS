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
from pyubx2 import UBXMessage, UBXReader
from pyubx2 import POLL

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from action_base.action import Base

from std_msgs.msg import String
from ubxmsg.msg import PVT, RELPOSNED, HPPOSLLH, VELNED, POSECEF

from rtk.ublox_functions import Ublox

class Rover(Node):
    def __init__(self):
        super().__init__('rover')
        self.uf = Ublox()

        self.PORT = "/dev/ttyACM0"
        # self.PORT = "/dev/serial0"

        self.BAUD = 57600
        self.TIMEOUT = 5
        self.stream = serial.Serial(self.PORT, self.BAUD, self.TIMEOUT)
        self.ubr = UBXReader(self.stream, protfilter=7) # 7 = UBS + NMEA + RTCM3
        self.stream.reset_input_buffer()

        # This is the port that the RTCM3 data will be emitted from
        self.RTCM3_PORT_TYPE = "UART2"  # choose from "USB", "UART1", "UART2"
        self.UBX_PORT_TYPE = "USB"

        self.TMODE = self.uf.TMODE_SVIN  # "TMODE_SVIN" or 1 = Survey-In, "TMODE_FIXED" or 2 = Fixed
        self.ACC_LIMIT = 5000  # accuracy in mm
        # only used if TMODE = 1 ...
        self.SVIN_MIN_DUR = 90  # seconds

        # only used if TMODE = 2 ... (Not Set Correctly Yet... These were the example defaults)
        self.ARP_LAT = 37.012345678
        self.ARP_LON = -115.012345678
        self.ARP_HEIGHT = 137000  # cm
        
        self.publisher_relposned = self.create_publisher(RELPOSNED, 'relposned_msg', 10)
        self.publisher_hpposllh = self.create_publisher(HPPOSLLH, 'hpposllh_msg', 10)
        self.publisher_velned = self.create_publisher(VELNED, 'velned_msg', 10)
        self.publisher_pvt = self.create_publisher(PVT, 'rover_pvt_msg', 10)
        self.publisher_posecef = self.create_publisher(POSECEF, 'posecef_msg', 10)

        
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
            pass
            # feedback_msg.progress = True
            # self.new_survay_in()
            # self.get_logger().info('Feedback: {0}'.format(feedback_msg.progress))

        if(goal_handle.request.action_id[0] == 2):
            self.ACC_LIMIT = goal_handle.request.action_id[1]
            feedback_msg.progress = True
            self.get_logger().info('New ACC LIMIT: {0}'.format(self.ACC_LIMIT))

        if(goal_handle.request.action_id[0] == 3):
            feedback_msg.progress = True
            # Turn Off All Messages
            self.uf.turn_off_all_msg(self.stream)
            time.sleep(0.1)
            self.get_logger().info('Turned All Messages Off ***')

        if(goal_handle.request.action_id[0] == 4):
            feedback_msg.progress = True
            # Turn Off All Messages
            self.port_setup()
            time.sleep(0.1)
            self.get_logger().info('Configured Ports for rtk robot stuff...')
            


        goal_handle.succeed()
        result = Base.Result()
        result.success = True
        return result

    def new_survay_in(self):
        # msg = self.uf.config_rtcm(self.RTCM3_PORT_TYPE)
        # self.uf.send_msg(self.stream, msg)
        self.uf.rtcm3_output(self.uf.RTCM3_List2, self.RTCM3_PORT_TYPE, self.stream)

        msg = self.uf.config_rover(self.RTCM3_PORT_TYPE, self.ACC_LIMIT, self.SVIN_MIN_DUR)
        self.uf.send_msg(self.stream, msg)
        time.sleep(1)
        msg = self.uf.config_svin(self.RTCM3_PORT_TYPE, self.ACC_LIMIT, self.SVIN_MIN_DUR)
        self.uf.send_msg(self.stream, msg)
    
    def port_setup(self):
        self.uf.rtcm3_output(self.uf.RTCM3_List2, self.RTCM3_PORT_TYPE, self.stream, 1)
        time.sleep(0.1)
        self.uf.ubx_nav_output(self.uf.NAV_List, self.UBX_PORT_TYPE, self.stream, 1)
        time.sleep(0.1)
        self.uf.ubx_nav_output(self.uf.NAV_List, self.RTCM3_PORT_TYPE, self.stream, 0)


    def timer_callback(self):
        msg_relposned = RELPOSNED()
        msg_hpposllh = HPPOSLLH() 
        msg_velned = VELNED()
        msg_pvt = PVT()
        msg_posecef = POSECEF()

        inWaiting_old = 0

        rec_relposned = False
        rec_hpposllh = False
        rec_velned = False
        rec_pvt = False
        rec_posecef = False
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

                if(parsed_data.identity == 'NAV-RELPOSNED'):
                    msg_relposned.time = parsed_data.iTOW
                    msg_relposned.rel_pos_north = float(parsed_data.relPosN)
                    msg_relposned.rel_pos_east = float(parsed_data.relPosE)
                    msg_relposned.rel_pos_down = float(parsed_data.relPosD)
                    msg_relposned.rel_pos_length = float(parsed_data.relPosLength)
                    msg_relposned.rel_pos_heading = float(parsed_data.relPosHeading)
                    msg_relposned.rel_pos_hpn = float(parsed_data.relPosHPN)
                    msg_relposned.rel_pos_hpe = float(parsed_data.relPosHPE)
                    msg_relposned.rel_pos_hpd = float(parsed_data.relPosHPD)
                    msg_relposned.rel_pos_hplength = float(parsed_data.relPosHPLength)
                    msg_relposned.acc_north = float(parsed_data.accN)
                    msg_relposned.acc_east = float(parsed_data.accE)
                    msg_relposned.acc_down = float(parsed_data.accD)
                    msg_relposned.acc_length = float(parsed_data.accLength)
                    msg_relposned.acc_heading = float(parsed_data.accHeading)
                    rec_relposned = True
                
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

                if(parsed_data.identity == 'NAV-VELNED'):
                    msg_velned.time = parsed_data.iTOW
                    msg_velned.v_north = float(parsed_data.velN)
                    msg_velned.v_east = float(parsed_data.velE)
                    msg_velned.v_down = float(parsed_data.velD)
                    msg_velned.speed_3d = float(parsed_data.speed)
                    msg_velned.speed_ground = float(parsed_data.gSpeed)
                    msg_velned.heading_2d = float(parsed_data.heading)
                    msg_velned.acc_vel_3d = float(parsed_data.sAcc)
                    msg_velned.acc_heading = float(parsed_data.cAcc)
                    rec_velned = True

                if(parsed_data.identity == 'NAV-HPPOSLLH'):
                    msg_hpposllh.time = parsed_data.iTOW
                    msg_hpposllh.lon = float(parsed_data.lon)
                    msg_hpposllh.lat = float(parsed_data.lat)
                    msg_hpposllh.h_ellipsoid = float(parsed_data.height)
                    msg_hpposllh.h_mean_sealevel = float(parsed_data.hMSL)
                    msg_hpposllh.h_geoid_sep = msg_hpposllh.h_ellipsoid - msg_hpposllh.h_mean_sealevel
                    msg_hpposllh.llh_invalid = bool(parsed_data.invalidLlh)
                    msg_hpposllh.acc_2d = float(parsed_data.hAcc)
                    msg_hpposllh.acc_vert = float(parsed_data.vAcc)
                    rec_hpposllh = True

                if(parsed_data.identity == 'NAV-POSECEF'):
                    msg_posecef.time = parsed_data.iTOW
                    msg_posecef.x_ecef = float(parsed_data.ecefX)
                    msg_posecef.y_ecef = float(parsed_data.ecefY)
                    msg_posecef.z_ecef = float(parsed_data.ecefZ)
                    msg_posecef.acc = float(parsed_data.pAcc)
                    rec_posecef = True
                    

                # print(parsed_data)
                # print('\n')
                if(self.stream.inWaiting() == inWaiting_old):
                    # print('Not Reading Anything')
                    break
                inWaiting_old = self.stream.inWaiting()

            print()
            if(rec_relposned):    
                self.publisher_relposned.publish(msg_relposned)
                print(msg_relposned)
                print()

            if(rec_velned):    
                self.publisher_hpposllh.publish(msg_hpposllh)
                # print(msg_hpposllh)

            if(rec_hpposllh):    
                self.publisher_velned.publish(msg_velned)
                # print(msg_velned)

            # if(rec_pvt):    
            #     self.publisher_pvt.publish(msg_pvt)
            #     print(msg_pvt)

            if(rec_posecef):
                self.publisher_posecef.publish(msg_posecef)
                print(msg_posecef)
                print()
            
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
