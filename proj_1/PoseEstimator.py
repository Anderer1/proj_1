import rclpy
import time
from rclpy.node import Node
import math

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
            Odometry,
            'odometry',
            self.odometry_callback,
            10)
        self.sub2 = self.create_subscription(
            PoseStamped,
            'GPSData',
            self.gpsdata_callback,
            10)           
        
        self.pub5 = self.create_publisher(PoseStamped, 'vehicle_pose', 10)
        self.subscription  # prevent unused variable warning
        self.x = 0
        self.y = 0        





    def gpsdata_callback(self, msg):

        self.x = msg.pose.position.x
        self.y = msg.pose.position.y 




        
    def odometry_callback(self, msg):
        

        msgEst = PoseStamped()
        theta = 2*math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
        deltaT = 0.15
        velocity = msg.twist.twist.linear.x 

        self.x = self.x + (velocity* math.cos(theta)*deltaT)
        
        self.y = self.y + (velocity* math.sin(theta)*deltaT)

        msgEst.pose.position.x =self.x 
        msgEst.pose.position.y =self.y 

        msgEst.pose.orientation.z = msg.pose.pose.orientation.z 

        msgEst.pose.orientation.w = msg.pose.pose.orientation.w 

        msgEst.header.stamp = self.get_clock().now().to_msg()
        
        self.pub5.publish(msgEst)


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