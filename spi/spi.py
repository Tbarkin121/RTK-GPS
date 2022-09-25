import time
# import serial
import spidev
import numpy as np

bus = 0
device = 0

spi = spidev.SpiDev()
spi.open(bus, device)

# Set SPI speed and mode
spi.max_speed_hz = 200000
spi.mode = 0

# to_send_next = np.array([0x01, 0x02, 0x03, 0x04], dtype=np.int16)
# print(to_send_next)
# bs = to_send_next.tobytes()
# for b in bs:
#     print(hex(b))

# to_send_next = np.array([-10000, -200, 300, 40000], dtype=np.int16)
# print(to_send_next)
# bs = to_send_next.tobytes()
# for b in bs:
#     print(hex(b))

to_send_next = np.array([0x00, 0x00, 0x00, 0x00], dtype=np.int16)
targ_vel_1 = np.int16(1000)
targ_vel_2 = np.int16(-1000)
targ_vel_3 = np.int16(2000)
targ_vel_4 = np.int16(-2000)
to_send_next[0] = targ_vel_1
to_send_next[1] = targ_vel_2
to_send_next[2] = targ_vel_3
to_send_next[3] = targ_vel_4
to_send = [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]

dt = 0.1
loop_count = 0
freq = 1/4
amp = 1000
while(1):
    targ_vel_1 = np.sin(2*np.pi*dt*loop_count*freq)*amp
    targ_vel_2 = -targ_vel_1
    targ_vel_3 = 2*targ_vel_1
    targ_vel_4 = -targ_vel_3
    to_send_next[0] = targ_vel_1
    to_send_next[1] = targ_vel_2
    to_send_next[2] = targ_vel_3
    to_send_next[3] = targ_vel_4

    bytes_to_send = to_send_next.tobytes()
    # print(to_send_next)
    for i in range(8):
        # print(hex(bytes_to_send[i]))
        to_send[i] = bytes_to_send[i]


    print('Sending SPI Message {}'.format(to_send_next))
    spi.xfer(to_send)


    time.sleep(dt)
    loop_count += 1