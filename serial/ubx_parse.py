import time
import serial
from pyubx2 import UBXReader


ser = serial.Serial("/dev/ttyACM0", 57600, timeout=3)
ubr = UBXReader(ser)
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
            (raw_data, parsed_data) = ubr.read()        
            print(parsed_data)
            print('\n')
        print('Data End:')
        print('\n\n\n')

        # ser.write(received_data)



        

