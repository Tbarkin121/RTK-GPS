from std_msgs.msg import ByteMultiArray
from std_msgs.msg import MultiArrayDimension

my_data = bytes('No Message', 'utf-8')
my_data2 = bytearray(my_data)

# print(my_data[0])
# print(my_data2[0])

msg = ByteMultiArray()
data_dim = MultiArrayDimension()
data_dim.label = "Test"
data_dim.size = 10
data_dim.stride = 1

print(msg)
print(msg.layout)
print(msg.layout.dim)
print(type(msg.layout.dim))

msg.layout.dim.append(data_dim)

msg.data=[my_data]

print(msg)
