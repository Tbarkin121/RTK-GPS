import rclpy
from rclpy.node import Node
from rclpy.serialization import serialize_message

# from std_msgs.msg import String
from ubxmsg.msg import SVIN, PVT

import rosbag2_py

class SimpleBagRecorder(Node):
    def __init__(self):
        super().__init__('simple_bag_recorder')
        self.writer = rosbag2_py.SequentialWriter()

        storage_options = rosbag2_py._storage.StorageOptions(
            uri='my_bag',
            storage_id='sqlite3')
        converter_options = rosbag2_py._storage.ConverterOptions('', '')
        self.writer.open(storage_options, converter_options)

        topic_info_svin = rosbag2_py._storage.TopicMetadata(
            name='svin_msg',
            type='ubxmsg/msg/SVIN',
            serialization_format='cdr')

        topic_info_pvt = rosbag2_py._storage.TopicMetadata(
            name='pvt_msg',
            type='ubxmsg/msg/PVT',
            serialization_format='cdr')

        self.writer.create_topic(topic_info_svin)
        self.writer.create_topic(topic_info_pvt)

        self.subscription_svin = self.create_subscription(
            SVIN,
            'svin_msg',
            self.topic_callback_svin,
            10)
        self.subscription_svin

        self.subscription_pvt = self.create_subscription(
            PVT,
            'pvt_msg',
            self.topic_callback_pvt,
            10)
        self.subscription_pvt

    def topic_callback_svin(self, msg):
        self.writer.write(
            'svin_msg',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)

    def topic_callback_pvt(self, msg):
        self.writer.write(
            'pvt_msg',
            serialize_message(msg),
            self.get_clock().now().nanoseconds)


def main(args=None):
    rclpy.init(args=args)
    sbr = SimpleBagRecorder()
    rclpy.spin(sbr)
    rclpy.shutdown()


if __name__ == '__main__':
    main()