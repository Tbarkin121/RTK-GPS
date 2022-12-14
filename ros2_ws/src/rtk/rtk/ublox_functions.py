import string
import random

import time
from math import trunc
from serial import Serial
from pyubx2 import UBXMessage, UBX_CONFIG_DATABASE

SHOW_PRESET = True  # hide or show PyGPSClient preset string
class Ublox(object):
    def __init__(self):
        print("RTK Base Station Init...")
        # initialise global variables
        self.reading = False

        self.TMODE_NONE = 0
        self.TMODE_SVIN = 1
        self.TMODE_FIXED = 2

        self.RTMC3_XXX4 = 0
        self.RTMC3_XXX7 = 1

        self.RTCM3_List1 = ["1005",
                            "1074",
                            "1084",
                            "1094",
                            "1124",
                            "1230",]
                    
        self.RTCM3_List2 = ["1005",
                            "1077",
                            "1087",
                            "1097",
                            "1127",
                            "1230",
                            "4072_0",
                            "4072_1",]
        
        self.NAV_List =     ["HPPOSECEF",
                             "HPPOSLLH",
                             "POSECEF",
                             "POSLLH",
                            #  "PVAT",
                             "PVT",
                             "RELPOSNED",
                             "SVIN",
                             "VELECEF",
                             "VELNED",
                            ]

        self.NAV_ListEx =   ["ODO",
                             "CLOCK",
                            #  "SOL", # Causing Message To Fail
                             "STATUS",
                            #  "PVAT", # Causing Message To Fail
                            ]

        self.HNR_List   =   ["CFG_MSGOUT_UBX_HNR_ATT",
                             "CFG_MSGOUT_UBX_HNR_INS",
                             "CFG_MSGOUT_UBX_HNR_PVT",
                            ]
        print("RTK Base Station Init...DONE")

    def send_msg(self, serial_out: Serial, ubx: UBXMessage):
        """
        Send config message to receiver.
        """

        print("Sending configuration message to receiver...")
        print(ubx)
        serial_out.write(ubx.serialize())

    def turn_off_all_msg(self, datastream):
        """
        Turn Off All Messages
        """
        layers = 1|2|4 # 1 = RAM, 2 = BBR, 4 = Flash (can be OR'd)
        transaction = 0
        cfg_data = []
        tuple_count = 0
        
        for (regName, addr) in UBX_CONFIG_DATABASE.items():
            if("MSGOUT" in regName):
            # if("CFG_MSGOUT_RTCM_3X_TYPE1230_UART2" in regName):
                print(regName)
                cfg_data.append([regName, 0])  
                tuple_count += 1
                if(tuple_count >= 16):
                    ubx = UBXMessage.config_set(layers, transaction, cfg_data)
                    self.send_msg(datastream, ubx)
                    cfg_data = []
                    tuple_count = 0 
                    time.sleep(0.01)
        # # Send remaining data
        ubx = UBXMessage.config_set(layers, transaction, cfg_data)
        self.send_msg(datastream, ubx)
    
    def rtcm3_output(self, rtcm3_list, port_type: str, datastream, val):
        """
        Configure which RTCM3 messages to output.
        """
        print("\nFormatting RTCM MSGOUT CFG-VALSET message...")
        layers = 1|2|4 # 1 = RAM, 2 = BBR, 4 = Flash (can be OR'd)
        transaction = 0
        cfg_data = []

        for rtcm_type in rtcm3_list:
            cfg = f"CFG_MSGOUT_RTCM_3X_TYPE{rtcm_type}_{port_type}"
            print(cfg)
            cfg_data.append([cfg, val])

        ubx = UBXMessage.config_set(layers, transaction, cfg_data)
        self.send_msg(datastream, ubx)
        if SHOW_PRESET:
            print(
                "Set ZED-F9P RTCM3 MSGOUT Basestation, "
                f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
            )

    def ubx_nav_output(self, ubx_list, port_type: str, datastream, val):
        """
        Configure which UBX messages to output.
        """
        
        layers = 1|2|4 # 1 = RAM, 2 = BBR, 4 = Flash (can be OR'd)
        transaction = 0
        cfg_data = []

        for ubx_type in ubx_list:
            cfg = f"CFG_MSGOUT_UBX_NAV_{ubx_type}_{port_type}"
            print(cfg)
            cfg_data.append([cfg, val])
        # cfg_data.append(["CFG_MSGOUT_UBX_NAV_SVIN_USB", val])
        # cfg_data.append(["CFG_MSGOUT_UBX_NAV_ODO_USB", val])
        # cfg_data.append(["CFG_MSGOUT_UBX_NAV_HPPOSLLH_USB", val])
        # cfg_data.append(["CFG_MSGOUT_UBX_NAV_POSLLH_USB", val])
        # cfg_data.append(["CFG_MSGOUT_UBX_NAV_HPPOSECEF_USB", val])
        # cfg_data.append(["CFG_MSGOUT_UBX_NAV_PVT_USB", val])

        ubx_out = UBXMessage.config_set(layers, transaction, cfg_data)
        self.send_msg(datastream, ubx_out)
        if SHOW_PRESET:
            print(
                "Set ZED-F9P RTCM3 MSGOUT Basestation, "
                f"CFG, CFG_VALSET, {ubx_out.payload.hex()}, 1\n"
            )


    def config_rtcm(self, port_type: str) -> UBXMessage:
        """
        Configure which RTCM3 messages to output.
        """

        print("\nFormatting RTCM MSGOUT CFG-VALSET message...")
        layers = 1|2|4 # 1 = RAM, 2 = BBR, 4 = Flash (can be OR'd)
        transaction = 0
        cfg_data = []

        for rtcm_type in (
            "1005",
            "1074",
            "1084",
            "1094",
            "1124",
            "1230",
        ):
            cfg = f"CFG_MSGOUT_RTCM_3X_TYPE{rtcm_type}_{port_type}"
            cfg_data.append([cfg, 0])

        for rtcm_type in (
            "1005",
            "1077",
            "1087",
            "1097",
            "1127",
            "1230",
            "4072_0",
            "4072_1",
        ):
            cfg = f"CFG_MSGOUT_RTCM_3X_TYPE{rtcm_type}_{port_type}"
            cfg_data.append([cfg, 1])

        ubx = UBXMessage.config_set(layers, transaction, cfg_data)

        if SHOW_PRESET:
            print(
                "Set ZED-F9P RTCM3 MSGOUT Basestation, "
                f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
            )

        return ubx

    def config_rover(self, port_type: str, acc_limit: int, svin_min_dur: int) -> UBXMessage:
        """
        Configure Survey-In mode with specied accuracy limit.
        """

        print("\nFormatting SVIN TMODE CFG-VALSET message...")
        tmode = self.TMODE_NONE
        layers = 1|2|4
        transaction = 0
        acc_limit = int(round(acc_limit / 0.1, 0))
        cfg_data = [
            ("CFG_TMODE_MODE", tmode),
            ("CFG_TMODE_SVIN_ACC_LIMIT", acc_limit),
            ("CFG_TMODE_SVIN_MIN_DUR", svin_min_dur),
            (f"CFG_MSGOUT_UBX_NAV_SVIN_{port_type}", 1),
        ]

        ubx = UBXMessage.config_set(layers, transaction, cfg_data)

        if SHOW_PRESET:
            print(
                "Set ZED-F9P to Survey-In Timing Mode Basestation, "
                f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
            )

        return ubx

    def config_svin(self, port_type: str, acc_limit: int, svin_min_dur: int) -> UBXMessage:
        """
        Configure Survey-In mode with specied accuracy limit.
        """

        print("\nFormatting SVIN TMODE CFG-VALSET message...")
        tmode = self.TMODE_SVIN
        layers = 1|2|4
        transaction = 0
        acc_limit = int(round(acc_limit / 0.1, 0))
        cfg_data = [
            ("CFG_TMODE_MODE", tmode),
            ("CFG_TMODE_SVIN_ACC_LIMIT", acc_limit),
            ("CFG_TMODE_SVIN_MIN_DUR", svin_min_dur),
            (f"CFG_MSGOUT_UBX_NAV_SVIN_{port_type}", 1),
        ]

        ubx = UBXMessage.config_set(layers, transaction, cfg_data)

        if SHOW_PRESET:
            print(
                "Set ZED-F9P to Survey-In Timing Mode Basestation, "
                f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
            )

        return ubx


    def config_fixed(self, acc_limit: int, lat: float, lon: float, height: float) -> UBXMessage:
        """
        Configure Fixed mode with specified coordinates.
        """

        print("\nFormatting FIXED TMODE CFG-VALSET message...")
        tmode = self.TMODE_FIXED
        pos_type = 1  # LLH (as opposed to ECEF)
        layers = 1
        transaction = 0
        acc_limit = int(round(acc_limit / 0.1, 0))

        # separate standard and high precision parts of lat / lon
        # and apply scaling factors
        lat_7dp = trunc(lat * 1e7) / 1e7
        lat_hp = lat - lat_7dp
        lat = int(round(lat_7dp / 1e-7, 0))
        lat_hp = int(round(lat_hp / 1e-9, 0))
        lon_7dp = trunc(lon * 1e7) / 1e7
        lon_hp = lon - lon_7dp
        lon = int(round(lon_7dp / 1e-7, 0))
        lon_hp = int(round(lon_hp / 1e-9, 0))

        height = int(height)
        cfg_data = [
            ("CFG_TMODE_MODE", tmode),
            ("CFG_TMODE_POS_TYPE", pos_type),
            ("CFG_TMODE_FIXED_POS_ACC", acc_limit),
            ("CFG_TMODE_HEIGHT_HP", 0),
            ("CFG_TMODE_HEIGHT", height),
            ("CFG_TMODE_LAT", lat),
            ("CFG_TMODE_LAT_HP", lat_hp),
            ("CFG_TMODE_LON", lon),
            ("CFG_TMODE_LON_HP", lon_hp),
        ]

        ubx = UBXMessage.config_set(layers, transaction, cfg_data)

        if SHOW_PRESET:
            print(
                "Set ZED-F9P to Fixed Timing Mode Basestation, "
                f"CFG, CFG_VALSET, {ubx.payload.hex()}, 1\n"
            )

        return ubx

        
    def talk(self):
        letters = string.ascii_lowercase
        random_letters = ''.join(random.choice(letters) for i in range(10))
        print(random_letters)
        return(random_letters)
