import time
import serial

ser = serial.Serial("/dev/serial0", 57600, timeout=1)  
# ser = serial.Serial("/dev/ttyAMA0", 115200, timeout=1)

ser.reset_input_buffer()

while True:
    ser.write(b'Hello World!!!')
    time.sleep(1)
