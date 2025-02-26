 #!/usr/bin/env python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from std_msgs.msg import Int16
from std_msgs.msg import Int16MultiArray
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import NavSatFix
from math import sin, cos, atan2, sqrt, degrees, pi, radians


from .my_ros_module import PubSubManager


class HeadingDecoder(Node):

    def __init__(self):
        super().__init__('heading_controller')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('rover_id', "p0")
            ]
        )
        self.target_rover = self.get_parameter('rover_id').value
        self.get_logger().info(f"Controller:{self.target_rover}")


        self.actual_heading_imu = None
        self.actual_heading_gps = None
        self.gps = []

        self.pubsub = PubSubManager(self)

        self.pubsub.create_subscription(
            Float32MultiArray,
            f'/{self.target_rover}/imu/eulerAngle',
            self.euler_callback, 
            5)
        self.pubsub.create_subscription(
            NavSatFix,
            f'/{self.target_rover}/gps1',
            self.current_gps_callback,
            5)

        self.pubsub.create_publisher(
            Int16,
            f'/heading/{self.target_rover}/actual',
            5)
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
    def current_gps_callback(self, msg_cur_pos:NavSatFix):
        self.status_gps = msg_cur_pos.status.status
        self.get_logger().info(f"[{self.target_rover}] Status GPS:", self.status_gps , "Lat/Lon:", msg_cur_pos.latitude,msg_cur_pos.longitude)
        if self.status_gps != 0:
            self.gps.append([msg_cur_pos.latitude,msg_cur_pos.longitude])
        if len(self.gps) > 1:
            self.actual_heading_gps = self.get_heading(self.gps[-1],self.gps[-2])
            self.get_logger().info(f"[{self.target_rover}] Status GPS:", self.status_gps , "Heading Angle:", self.actual_heading_gps)
            msg = Int16()
            msg.data = self.actual_heading_gps
            self.pubsub.publish(f'/heading/{self.target_rover}/actual',msg)
            if len(self.gps) > 10:
                self.gps.pop(0)
    
    def get_heading(self, pos1, pos2):
        lat1, lon1 = pos1
        lat2, lon2 = pos2
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        y = sin(dlon) * cos(lat2)
        x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(dlon)
        return (degrees(atan2(y, x)) + 360) % 360


    def euler_callback(self, msg_imu_euler:Float32MultiArray):
        self.actual_heading_imu =  msg_imu_euler.data[0]
        self.get_logger().info(f"[{self.target_rover}] Status IMU:", self.target_rover , "Heading Angle:", self.cur_heading)
    
    def timer_callback(self):
        if self.actual_heading_imu is not None and self.actual_heading_gps is not None:
            self.get_logger().info(f"[{self.target_rover}] Heading IMU:{self.actual_heading_imu} Heading GPS:{self.actual_heading_gps}")
            if abs(self.actual_heading_imu - self.actual_heading_gps) < 10:
                self.actual_heading = (self.actual_heading_imu + self.actual_heading_gps) / 2
            else:
                self.actual_heading = self.actual_heading_imu
            msg = Int16()
            msg.data = self.actual_heading
            self.pubsub.publish(f'/heading/{self.target_rover}/actual',msg)
        else:
            self.get_logger().info(f"[{self.target_rover}] Heading IMU:{self.actual_heading_imu} Heading GPS:{self.actual_heading_gps}")

def main(args=None):
    rclpy.init(args=args)

    heading_decoder = HeadingDecoder()
    rclpy.spin(heading_decoder)
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    heading_decoder.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
