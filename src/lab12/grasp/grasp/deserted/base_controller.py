import time
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist, Pose

from grasp_interfaces.srv import GraspAction, GraspQuery


class PlanarPose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

    def __str__(self):
        return f'{{ x: {self.x}, y: {self.y}, theta: {self.theta} }}'


class BaseController(Node):
    def __init__(self):
        super().__init__('BaseController')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1024)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        
        self.current_pose: Pose = None
        self.pose: PlanarPose = None
        
        self.current_goal: Pose = None
        self.goal: PlanarPose = None
        self.goal_stage = "FACETO" # FACETO, GO, TURNTO, FINISH
        
        self.pose_stamp = None
        self.pose_expired_time = 1000 # TODO
        
        self.linear_kp = 0.2
        self.linear_kd = 0.08
        self.angular_kp = 0.35
        self.angular_kd = 0.03
        self.last_vel = Twist()
        
        self.thred_theta = 2 / 180 * math.pi
        self.thred_linear = 0.05
        
        # use initial pose as the first current_pose
        # self.amcl_pose_callback(initial_pose)
    
    def amcl_pose_callback(self, msg):
        # self.get_logger().info(f'{str(msg)}')
        self.current_pose = msg.pose.pose
        self.pose = self.pose_to_xytheta(msg.pose.pose)
        self.pose_stamp = msg.header.stamp
    
    def pub_cmd_vel(self, twist):
        self.get_logger().info(f'{str(twist)}')
        self.cmd_vel_pub.publish(twist)
        self.last_vel = twist
    
    def quaternion_to_yaw(self, quaternion):
        q_0, q_1, q_2, q_3 = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        return math.atan2(2 * (q_0 * q_3 + q_1 * q_2), 1 - 2 * (q_2 * q_2 + q_3 * q_3))
    
    def pose_to_xytheta(self, pose):
        x = pose.position.x
        y = pose.position.y
        theta = self.quaternion_to_yaw(pose.orientation)
        return PlanarPose(x, y, theta)
    
    def set_goal(self, pose):
        self.current_goal = pose
        self.goal = self.pose_to_xytheta(pose)
        self.goal_stage = "FACETO"
    
    def is_goal_reached(self):
        return self.goal_stage == "FINISH"
    
    def get_dtheta(self, theta, target_theta):
        if target_theta > theta:
            target_theta_max = target_theta
            target_theta_min = target_theta - 2 * math.pi
        else:
            target_theta_max = target_theta + 2 * math.pi
            target_theta_min = target_theta
        
        dtheta_ccw = target_theta_max - theta # +
        dtheta_cw = target_theta_min - theta # -
        return dtheta_ccw if dtheta_ccw < -dtheta_cw else dtheta_cw

    def move_control(self, x, y, allow_move = True, theta_check = True):
        sec, _ = self.get_clock().now().seconds_nanoseconds()
        if self.pose_stamp is None or self.pose_stamp.sec - sec > self.pose_expired_time:
            return
        
        dx = x - self.pose.x
        dy = y - self.pose.y
        d = math.sqrt(dx * dx + dy * dy)
        dtheta = self.get_dtheta(self.pose.theta, math.atan2(dy, dx))
        
        cmd = Twist()
        if abs(dtheta) > self.thred_theta:
            cmd.angular.z = self.angular_kp * dtheta - self.angular_kd * self.last_vel.angular.z
        if allow_move and d > self.thred_linear:
            cmd.linear.x = self.linear_kp * d - self.linear_kd * self.last_vel.linear.x
        self.pub_cmd_vel(cmd)
        
        return (not theta_check or abs(dtheta) <= self.thred_theta) and (not allow_move or d <= self.thred_linear)
    
    def angle_control(self, theta):
        dtheta = self.get_dtheta(self.pose.theta, theta)
        
        cmd = Twist()
        if abs(dtheta) > self.thred_theta:
            cmd.angular.z = self.angular_kp * dtheta - self.angular_kd * self.last_vel.angular.z
        self.pub_cmd_vel(cmd)
        
        return abs(dtheta) <= self.thred_theta
    
    def velocity_controller(self):
        if self.current_pose is None:
            self.get_logger().info('waiting for amcl_pose ...')
            return
        if self.current_goal is not None:
            self.get_logger().info(f'Stage {self.goal_stage}: {self.pose}, {self.goal}')
            
            if self.goal_stage == "FACETO":
                if self.move_control(self.goal.x, self.goal.y, allow_move = False, theta_check = True):
                    self.goal_stage = "GO"
            elif self.goal_stage == "GO":
                if self.move_control(self.goal.x, self.goal.y, allow_move = True, theta_check = False):
                    self.goal_stage = "TURNTO"
            elif self.goal_stage == "TURNTO":
                if self.angle_control(self.goal.theta):
                    self.goal_stage = "FINISH"

