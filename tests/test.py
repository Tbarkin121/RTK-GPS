from std_msgs.msg import ByteMultiArray

msg = ByteMultiArray()
msg.data = [bytes('No Message', 'utf-8')]

print(msg.data)
