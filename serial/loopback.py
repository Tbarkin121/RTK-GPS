import serial
from time import sleep

ser = serial.Serial ("/dev/serial0", 57600)    #Open port with baud rate
while True:
    received_data = ser.read()              #read serial port
    sleep(0.5)
    data_left = ser.inWaiting()             #check for remaining byte
    received_data += ser.read(data_left)
    print (received_data)                   #print received data
    print()
    print(len(received_data))
    print()
    ser.write(received_data)                #transmit data serially 