import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy
from std_msgs.msg import Bool, Int16
from std_msgs.msg import String

from .my_ros_module import PubSubManager


class Demux(Node):
    def __init__(self, n_rover=6):
        super().__init__(f'demux')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('robot_id_list', ["p0"]),
                ('mode', "Joy"),
                ('heading_controller', False)
            ]
        )
        params = self._parameters
        for name, param in params.items():
            self.get_logger().info(f"PARAMS/ {name}: {param.value}")
        self.mode = self.get_parameter('mode').value
        self.block = '' if self.mode == "Joy" else '/joy'
        self.select = 0
        
        robot_id_list = self.get_parameter('robot_id_list').value
        self.get_logger().info(f"robot_id_list: {robot_id_list}")
        if len(robot_id_list) == 1 and robot_id_list[0] == "p0":
            robot_id_list = None
        self.robot_id_list = []
        if robot_id_list is None:
            self.n_rover = n_rover
            for i in range(self.n_rover):
                self.robot_id_list.append(f"{self.robot_id_list[i]}")
        elif isinstance(robot_id_list, int):
            self.n_rover = n_rover
            for i in range(self.n_rover):
                self.robot_id_list.append(f"p{i+1+robot_id_list}")
        elif isinstance(robot_id_list, list):
            self.n_rover = len(robot_id_list)
            if isinstance(robot_id_list[0], int):
                for i in range(self.n_rover):
                    self.robot_id_list.append(f"p{robot_id_list[i]}")
            else:
                self.robot_id_list = robot_id_list
        else:
            print("ROBOT_ID_LIST ERROR")
            return
        self.get_logger().info(f"ROVER: {self.robot_id_list} N: {self.n_rover}")


        self.heading_controller = self.get_parameter('heading_controller').value
        self.angular_vel = [0.0] * self.n_rover

        self.broadcast = False

        self.hardware = True

        self.pubsub = PubSubManager(self)
        self.pubsub.create_subscription(
            Twist,
            '/joy/cmd_vel',
            self.joy_cmd_callback,
            5)
        self.pubsub.create_subscription(
            Bool,
            '/joy/enable',
            self.joy_en_callback,
            5)
        self.pubsub.create_subscription(
            Int16,
            '/select_rover',
            self.sel_callback,
            1)
        self.pubsub.create_subscription(
            Bool,
            '/joy/broadcast',
            self.broadcast_callback,
            1)
        self.pubsub.create_subscription(
            Bool,
            '/joy/hardware',
            self.hw_sim_callback,
            1)
        
        for i in range(self.n_rover):
            self.pubsub.create_publisher(Twist, f'{self.block}/{self.robot_id_list[i]}/cmd_vel', 5)
            self.pubsub.create_publisher(Bool, f'{self.block}/{self.robot_id_list[i]}/enable', 5)
            if self.heading_controller:
                self.pubsub.create_subscription(
                    Twist,
                    f'/heading/{self.robot_id_list[i]}/anglar_vel',
                    lambda msg, i=i: self.angular_callback(msg, i),
                    5)
                self.pubsub.create_publisher(Int16, f'/heading/{self.robot_id_list[i]}/desired', 5)
        
        if self.heading_controller:
            self.pubsub.create_subscription(
                Int16,
                '/joy/angle_sel',
                self.angle_sel_callback,
                5)
        self.pubsub.create_publisher(Twist, '/sim/cmd_vel', 10)
        self.pubsub.create_publisher(Bool, '/sim/enable', 10)
        
    
    def joy_cmd_callback(self, msg):
        self.lx = msg.linear.x
        self.az = msg.angular.z

    def broadcast_callback(self, msg):
        self.broadcast = msg.data

    def hw_sim_callback(self, msg):
        self.hardware = msg.data
    
    def sel_callback(self, msg):
        self.select = msg.data
        
    def joy_en_callback(self,msg):
        _en = msg.data
        val = Twist()           # For selected rover
        en_state = Bool()       # For selected rover
        empty_twist = Twist()   # For other rovers
        false_state = Bool()    # For other rovers
        empty_twist.linear.x = 0.0
        empty_twist.angular.z = 0.0
        false_state.data = False

        en_state.data = True
        val.linear.x = self.lx
        val.angular.z = self.az

        # Sim
        if not self.hardware:
            self.pubsub.publish(f'/sim/cmd_vel', val)
            self.pubsub.publish(f'/sim/enable', en_state)
        
        for i in range(self.n_rover):
            if not self.hardware:
                self.pubsub.publish(f'{self.block}/{self.robot_id_list[i]}/cmd_vel', empty_twist)
                self.pubsub.publish(f'{self.block}/{self.robot_id_list[i]}/enable', false_state)
            elif self.broadcast:
                en_state.data = True
                if self.heading_controller:
                    val.angular.z = self.angular_vel[i]
                self.pubsub.publish(f'{self.block}/{self.robot_id_list[i]}/cmd_vel', val)
                self.pubsub.publish(f'{self.block}/{self.robot_id_list[i]}/enable', en_state)
            elif _en and self.select == i+1:
                en_state.data = True
                if self.heading_controller:
                    val.angular.z = self.angular_vel[i]
                self.pubsub.publish(f'{self.block}/{self.robot_id_list[i]}/cmd_vel', val)
                self.pubsub.publish(f'{self.block}/{self.robot_id_list[i]}/enable', en_state)
            else:
                self.pubsub.publish(f'{self.block}/{self.robot_id_list[i]}/cmd_vel', empty_twist)
                self.pubsub.publish(f'{self.block}/{self.robot_id_list[i]}/enable', false_state)  

    def angular_callback(self, msg, i):
        self.angular_vel[i] = msg.angular.z
        self.get_logger().info(f"{self.robot_id_list[i]}/ Received angular vel: {msg.angular.z}")   

    def angle_sel_callback(self, msg):
        angle_msg = Int16()
        angle_msg.data = msg.data
        for i in range(self.n_rover):
            self.pubsub.publish(f'/heading/{self.robot_id_list[i]}/desired', angle_msg)
        self.get_logger().info(f"Angle selected: {msg.data}")    


def main(args=None):
    rclpy.init(args=args)
    demultiplexer = Demux()
    rclpy.spin(demultiplexer)
    demultiplexer.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#ros2 run joy joy_node
