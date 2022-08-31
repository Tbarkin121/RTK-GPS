from std_msgs.msg import ByteMultiArray

msg = ByteMultiArray()
my_data = bytes('No Message', 'utf-8')
print(my_data)
print(len(my_data))
print(type(my_data))
print('First Element:')
print(my_data[0])
print(type(my_data[0]))
msg.data = [my_data]

print()
print('Message Data:')
print(msg.data)
print(len(msg.data))
print('First Message Entry:')
print(msg.data[0])
print(len(msg.data[0]))
print(type(msg.data[0]))
print('First Entry of First Message')
print(msg.data[0][0])
print(type(msg.data[0][0]))

