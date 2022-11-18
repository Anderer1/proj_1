import rclpy
import time
from rclpy.node import Node

from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from drive_interfaces.msg import VehCmd
from nav_msgs import Odometry



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
        self.pub3 = self.create_publisher(Float32, 'error', 10)
        self.pub4 = self.create_publisher(Float32, 'velocity_output', 10)
        self.subscription  # prevent unused variable warning


    def listener_callback(self, msg):
        comang = VehCmd()
        count = 0
        if msg.buttons[1] == 1:
            comang.throttle_effort = 0.0
        if msg.buttons[0] == 1:
            if count == 0:
                comang.throttle_effort = 0.25*100*7.3513268
                count = 1
            else: 
                comang.throttle_effort = 0.0
                count = 0
        Kp = 0.01
        Ki = 0.0
        Kd = 0.0

        #r = msg.axes[1]*100*7.3513268 #the 7.35 is the max velocity in m/s
        #y = msg.twist.x 
        #et = r - y
        et = (msg.axes[1]*100*7.3513268)  - msg.twist.x 
        print(et)
        self.etlast = et
        deltaT = 0.1
        integ =  et*deltaT
        self.oldinteg = 0;
        self.oldinteg = self.oldinteg + integ
        comang.throttle_effort = (Kp*et) + Ki*(self.oldinteg) + (Kd*((et -self.etlast)/deltaT))
        comang.steering_angle = msg.axes[3]*45
        led = Int16()
        led.data = 1

        self.pub1.publish(led)
        self.pub2.publish(comang)
        self.pub3.publish(et)
        self.pub4.publish(comang.throttle_effort)
        

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