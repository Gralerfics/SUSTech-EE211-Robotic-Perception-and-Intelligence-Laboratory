import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Bot(Node):
    def __init__(self, node_name = 'bot_ctrl'):
        super().__init__(node_name)
        
        self.pub_cmd_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub_odom = self.create_subscription(Odometry, 'odom', self.sub_odom_callback, QoSProfile(depth = 10, reliability = ReliabilityPolicy.BEST_EFFORT))
        
        self.odom = None
    
    def sub_odom_callback(self, msg):
        self.odom = msg
    
    def go_forward(self, dist): # , speed = None):
        initial_x = self.odom.pose.pose.position.x
        initial_y = self.odom.pose.pose.position.y
        distance_moved = 0.0
        
        cmd_vel = Twist()
        cmd_vel.linear.x = 0.0
        while distance_moved < dist:
            self.pub_cmd_vel.publish(cmd_vel)
            e_x = self.odom.pose.pose.position.x - initial_x
            e_y = self.odom.pose.pose.position.y - initial_y
            distance_moved = math.sqrt(e_x * e_x + e_y * e_y)
            
            e = dist - distance_moved
            e_dot = -cmd_vel.linear.x
            
            k_p = 0.5
            k_d = 0.0
            cmd_vel.linear.x = k_p * e + k_d * e_dot
            
            rclpy.spin_once(self)
        
        cmd_vel.linear.x = 0.0
        self.pub_cmd_vel.publish(cmd_vel)

    def turn(self, angle, angular_speed = 0.3):
        initial_yaw = self.quaternion_to_yaw(self.odom.pose.pose.orientation)
        target_yaw = initial_yaw + angle
        if target_yaw <= -math.pi:
            target_yaw += 2 * math.pi
        cmd_vel = Twist()

        while abs(self.quaternion_to_yaw(self.odom.pose.pose.orientation) - target_yaw) > 0.05:
            cmd_vel.angular.z = angular_speed if angle > 0 else -angular_speed
            self.pub_cmd_vel.publish(cmd_vel)
            
            rclpy.spin_once(self)

        cmd_vel.angular.z = 0.0
        self.pub_cmd_vel.publish(cmd_vel)
    
    def quaternion_to_yaw(self, quaternion):
        q_0, q_1, q_2, q_3 = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        return math.atan2(2 * (q_0 * q_3 + q_1 * q_2), 1 - 2 * (q_2 * q_2 + q_3 * q_3))


def main(args = None):
    rclpy.init(args = args)
    
    try:
        move_square = Bot('move_square')
        
        while not move_square.odom:
            rclpy.spin_once(move_square)
        
        move_square.go_forward(0.3)
        move_square.turn(-math.pi / 2)
        move_square.go_forward(0.15)
        move_square.turn(-math.pi / 2)
        move_square.go_forward(0.3)
        move_square.turn(-math.pi / 2)
        move_square.go_forward(0.15)
        move_square.turn(-math.pi / 2)
        
    finally:
        move_square.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

