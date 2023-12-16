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
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        
        self.motion_timer = self.create_timer(0.2, self.motion_timer_callback)
        self.fsm_timer = self.create_timer(0.2, self.fsm_timer_callback)
        
        self.state = 'INIT'
        self.is_goal_finished = False
        self.goal_stage = 0
        
        self.current_pose: Pose = None
        self.current_goal: Pose = None
        self.pose: PlanarPose = None
        self.goal: PlanarPose = None
        
        self.pose_stamp = None
        self.pose_expired_time = 1000 # TODO
        
        self.linear_kp = 0.1
        self.linear_kd = 0.01
        self.angular_kp = 0.35
        self.angular_kd = 0.01
        
        self.thred_theta = 2 / 180 * math.pi
        self.thred_linear = 0.05
        
        # publish initial pose
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.pose.position.x = 0.3025282477783394
        initial_pose.pose.pose.position.y = -0.36741189949949404
        initial_pose.pose.pose.orientation.z = -0.7140501663813629
        initial_pose.pose.pose.orientation.w = 0.7000945363954414
        self.initialpose_pub.publish(initial_pose)
        
        # use initial pose as the first current_pose
        self.amcl_pose_callback(initial_pose)
    
    def pub_cmd_vel(self, twist):
        self.get_logger().info(f'{str(twist)}')
        self.cmd_vel_pub.publish(twist)
    
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
        self.is_goal_finished = False
        self.goal_stage = 0
    
    def amcl_pose_callback(self, msg):
        self.current_pose = msg.pose.pose
        self.pose = self.pose_to_xytheta(msg.pose.pose)
        self.pose_stamp = msg.header.stamp
    
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

    def move_control(self, x, y, allow_move = True):
        sec, _ = self.get_clock().now().seconds_nanoseconds()
        if self.pose_stamp is None or self.pose_stamp.sec - sec > self.pose_expired_time:
            return
        
        dx = x - self.pose.x
        dy = y - self.pose.y
        d = math.sqrt(dx * dx + dy * dy)
        dtheta = self.get_dtheta(self.pose.theta, math.atan2(dy, dx))
        
        cmd = Twist()
        if abs(dtheta) > self.thred_theta:
            cmd.angular.z = self.angular_kp * dtheta
        if allow_move and d > self.thred_linear:
            cmd.linear.x = self.linear_kp * d
        self.pub_cmd_vel(cmd)
        
        return abs(dtheta) <= self.thred_theta and (not allow_move or d <= self.thred_linear)
    
    def angle_control(self, theta):
        dtheta = self.get_dtheta(self.pose.theta, theta)
        
        cmd = Twist()
        if abs(dtheta) > self.thred_theta:
            cmd.angular.z = self.angular_kp * dtheta
        self.pub_cmd_vel(cmd)
        
        return abs(dtheta) <= self.thred_theta
    
    def motion_timer_callback(self):
        if self.current_pose is None:
            return
        if self.current_goal is not None:
            self.get_logger().info(f'Stage {self.goal_stage}: {self.pose}, {self.goal}')
            
            if self.goal_stage == 0:
                if self.move_control(self.goal.x, self.goal.y, allow_move = False):
                    self.goal_stage = 1
            elif self.goal_stage == 1:
                if self.move_control(self.goal.x, self.goal.y):
                    self.goal_stage = 2
            elif self.goal_stage == 2:
                if self.angle_control(self.goal.theta):
                    self.goal_stage = 3
                    self.is_goal_finished = True

    def fsm_timer_callback(self):
        if self.state == 'INIT':
            Af = Pose()
            Af.position.x = 0.3066274822848437
            Af.position.y = -3.1658465986026695
            # Af.position.y = -3.2658465986026695
            Af.orientation.z = -0.6686671777907486
            Af.orientation.w = 0.7435618369344646
            self.set_goal(Af)
            self.state = 'StoA'
        elif self.state == 'StoA':
            self.get_logger().info('Go to A from S ...')
            if self.is_goal_finished:
                self.get_logger().info('Done.')
                # grasp
        else:
            self.get_logger().info('Invalid state.')

