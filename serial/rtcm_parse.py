import time
import serial
from pyrtcm import RTCMReader

ser = serial.Serial("/dev/serial0", 57600, timeout=3)
rtr = RTCMReader(ser)
ser.reset_input_buffer()

while(1):
    inWaiting_old = 0
    if ser.inWaiting()>0:
        inWaiting_old = ser.inWaiting()
        print('ser.in_waiting : {}'.format(ser.inWaiting()))		
        while (inWaiting_old != ser.inWaiting()):
            inWaiting_old = ser.inWaiting()
            time.sleep(0.1)
    
        data_left = ser.inWaiting()             #check for remaining byte
        print('FRESH DATA, GET EM WHILE ITS HOT!')
        while(ser.inWaiting()):
            print('data in waiting : {}'.format(ser.inWaiting()))    
            (raw_data, parsed_data) = rtr.read()        
            print(parsed_data)
            print('\n')
        print('Data End:')
        print('\n\n\n')

        # ser.write(received_data)



        

