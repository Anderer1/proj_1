import rclpy
from rclpy.node import Node

from std_msgs.msg import Int16
from sensor_msgs.msg import Joy
from drive_interfaces.msg import VehCmd


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.pub1 = self.create_publisher(Int16, 'led_color', 10)
        self.pub2 = self.create_publisher(VehCmd, 'vehicle_command_angle', 10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        comang = VehCmd()
        if msg.buttons[2] == 1:
            comang.throttle_effort = 0
        comang.throttle_effort = msg.axes[2]*100
        comang.steering_angle = msg.axes[4]*45
        led = Int16()
        led.data = 1

        self.pub1.publish(led)
        self.pub2.publish(comang)

def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()