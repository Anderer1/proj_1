import rclpy
import time
from rclpy.node import Node

from std_msgs.msg import Int16
from std_msgs.msg import Float32
from sensor_msgs.msg import Joy
from drive_interfaces.msg import VehCmd
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped



class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.listener_callback,
            10)
        self.subscription = self.create_subscription(
            Odometry,
            'odometry',
            self.odometry_callback,
            10)
        self.subscription = self.create_subscription(
            GPSData,
            'gpsdata',
            self.odometry_callback,
            10)           
        self.pub1 = self.create_publisher(Int16, 'led_color', 10)
        self.pub2 = self.create_publisher(VehCmd, 'vehicle_command_angle', 10)
        self.pub3 = self.create_publisher(Float32, 'error', 10)
        self.pub4 = self.create_publisher(Float32, 'velocity_output', 10)
        self.pub5 = self.create_publisher(Float32, 'vehicle_pose', 10)
        self.subscription  # prevent unused variable warning
        self.comang = VehCmd()
        self.oldthrottle = 0.0
        self.desiredspeed = 0.0
        self.etlast = 0.0

        led = Int16()
        led.data = 1

        self.pub1.publish(led)


    def listener_callback(self, msg):
        if msg.buttons[1] == 1:
            self.desiredspeed = 0.0
        if msg.buttons[0] == 1:
            self.desiredspeed = 0.25*7.3513268
        #msg.axes[1]*100*7.3513268   should be desired speed in class variable     
        self.comang.steering_angle = msg.axes[3]*45

        
    def odometry_callback(self, msg):

        Kp = 1.5
        Ki = 0.0
        Kd = 0.1

        #r = msg.axes[1]*100*7.3513268 #the 7.35 is the max velocity in m/s
        #y = msg.twist.x 
        #et = r - y
        et = (self.desiredspeed)  - msg.twist.twist.linear.x 
        deltaT = 0.15
        integ =  et*deltaT
        self.oldinteg = 0
        self.oldinteg = self.oldinteg + integ
        self.comang.throttle_effort = (Kp*et) + Ki*(self.oldinteg) + (Kd*((et -self.etlast)/deltaT)) + self.oldthrottle
        self.etlast = et
        if self.comang.throttle_effort < 0.0:
            self.comang.throttle_effort = 0.0
        elif self.comang.throttle_effort > 100:
            self.comang.throttle_effort = 100.0
        print(f"Desired Speed: {self.desiredspeed}, Actual Speed: {msg.twist.twist.linear.x}, Effort: {self.comang.throttle_effort}, Error: {et}")
        self.oldthrottle = self.comang.throttle_effort
        
        self.pub2.publish(self.comang)
        et_message = Float32()
        et_message.data = et
        self.pub3.publish(et_message)
        throttle_message = Float32()
        throttle_message.data = self.comang.throttle_effort
        self.pub4.publish(throttle_message)
        
        

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