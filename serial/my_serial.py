import time
import serial

ser = serial.Serial("/dev/serial0", 57600)

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
		print(type(received_data))
		print(received_data)
		int_data = []
		for ints in received_data:
			int_data.append(ints)

		print()
		print(int_data)
		back_to_bytes = []
		for i in int_data:
			back_to_bytes.append(i.to_bytes(1, byteorder='little'))
		print()
		print(back_to_bytes)

		# print(type(received_data.decode("utf_32") ))

		# ser.write(received_data)



		

