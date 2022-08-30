import time
import serial

ser = serial.Serial("/dev/ttyACM0", 57600)

ser.reset_input_buffer()

while True:
	inWaiting_old = 0
	if ser.inWaiting()>0:
		inWaiting_old = ser.inWaiting()
		print('ser.in_waiting : {}'.format(ser.inWaiting()))		
		received_data = ser.read()
		# time.sleep(0.25)
		while (inWaiting_old != ser.inWaiting()):
			inWaiting_old = ser.inWaiting()
			time.sleep(0.05)
	
		data_left = ser.inWaiting()             #check for remaining byte
		received_data += ser.read(data_left)

		print('Data Len : {}'.format(len(received_data)))
		print(received_data)
		# print(type(received_data ))

		# ser.write(received_data)



		

