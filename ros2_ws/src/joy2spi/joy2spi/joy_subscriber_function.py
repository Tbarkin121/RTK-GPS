import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy

# For SPI Communication
import time
import spidev
import numpy as np

class JoySubscriber(Node):

    def __init__(self):
        super().__init__('joystick_subscriber')
        self.get_logger().info('Starting Joy Subscriber Node')

        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Setting Up Timer for SPI Transfer Rate Control
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.spi_timer_callback)

        # Setting up SPI Bus
        bus = 0
        device = 0
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 200000
        self.spi.mode = 0
        self.to_send_next = np.array([0x00, 0x00, 0x00, 0x00], dtype=np.int16)
        self.to_send = [0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0]
        targ_vel_1 = np.int16(0)
        targ_vel_2 = np.int16(0)
        targ_vel_3 = np.int16(0)
        targ_vel_4 = np.int16(0)
        self.to_send_next[0] = targ_vel_1
        self.to_send_next[1] = targ_vel_2
        self.to_send_next[2] = targ_vel_3
        self.to_send_next[3] = targ_vel_4
        
    def spi_timer_callback(self):
        bytes_to_send = self.to_send_next.tobytes()
        for i in range(8):
            self.to_send[i] = bytes_to_send[i]
        # print('Sending SPI Message {}'.format(self.to_send_next))
        self.spi.xfer(self.to_send)

    def listener_callback(self, msg):
        self.get_logger().info('Msg Received')
        # print("Axis 1 : {}".format(msg.axes[0])) # X Axis
        # print("Axis 2 : {}".format(msg.axes[1])) # Y Axis
        MAX_VEL = 1000
        targ_vel_1 = MAX_VEL*msg.axes[0]
        targ_vel_2 = -MAX_VEL*msg.axes[0]
        targ_vel_3 = MAX_VEL*msg.axes[1]
        targ_vel_4 = -MAX_VEL*msg.axes[1]
        self.to_send_next[0] = targ_vel_1
        self.to_send_next[1] = targ_vel_2
        self.to_send_next[2] = targ_vel_3
        self.to_send_next[3] = targ_vel_4
        


def main(args=None):
    rclpy.init(args=args)

    joy_subscriber = JoySubscriber()

    rclpy.spin(joy_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    joy_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
