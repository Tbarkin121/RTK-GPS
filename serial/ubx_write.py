import time
import serial
from pyubx2 import UBXReader


ser = serial.Serial("/dev/ttyACM0", 57600, timeout=3)
# ubr = UBXReader(ser, protfilter=3) # NMEA and UBX
ubr = UBXReader(ser, protfilter=2) # Just UBS

ser.reset_input_buffer()

message_recieved = False
while(1):
    inWaiting_old = 0
    if ser.inWaiting()>0:
        inWaiting_old = ser.inWaiting()
        print('ser.in_waiting : {}'.format(ser.inWaiting()))		
        while (inWaiting_old != ser.inWaiting()):
            inWaiting_old = ser.inWaiting()
            time.sleep(0.1)
    
        inWaiting_old = ser.inWaiting()
        print('FRESH DATA, GET EM WHILE ITS HOT!')
        while(ser.inWaiting()):
            print('data in waiting : {}'.format(ser.inWaiting()))    
            (raw_data, parsed_data) = ubr.read()
            print(type(parsed_data))
            print(parsed_data.identity)
            if(parsed_data.identity == 'NAV-POSLLH'):
                print('Lat : {}'.format(parsed_data.lat))
                print('Lon : {}'.format(parsed_data.lon))       
            print(parsed_data)
            print('\n')
            if(ser.inWaiting() == inWaiting_old):
                # print('Not Reading Anything')
                break
            inWaiting_old = ser.inWaiting()

        print('Data End:')
        print('\n\n\n')
        message_recieved = True
        break

    # if(message_recieved):
    #     break

        # ser.write(received_data)



        

