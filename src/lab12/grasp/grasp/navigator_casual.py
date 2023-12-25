import time
import math

import numpy as np
import tf_transformations

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from geometry_msgs.msg import Pose, PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import Odometry
from pan_tilt_msgs.msg import PanTiltCmdDeg
from std_msgs.msg import Int32MultiArray

from grasp_interfaces.srv import GraspAction, GraspQuery

from .tf_reader import TF_Reader


def transform_to_matrix(trans):
    translation = np.array([trans.translation.x, trans.translation.y, trans.translation.z])
    rotation = [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]
    R = tf_transformations.quaternion_matrix(rotation)
    matrix = np.eye(4)
    matrix[:3, :3] = R[:3, :3]
    matrix[:3, 3] = translation
    return matrix

def pose_to_matrix(pose):
    translation = np.array([pose.position.x, pose.position.y, pose.position.z])
    rotation = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    R = tf_transformations.quaternion_matrix(rotation)
    matrix = np.eye(4)
    matrix[:3, :3] = R[:3, :3]
    matrix[:3, 3] = translation
    return matrix

def matrix_to_pose(matrix):
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = matrix[0, 3], matrix[1, 3], matrix[2, 3]
    quat = tf_transformations.quaternion_from_matrix(matrix)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat[0], quat[1], quat[2], quat[3]
    return pose


class PIDController:
    def __init__(self, Kp, Ki, Kd, min_output = None, max_output = None):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.min_output = min_output
        self.max_output = max_output
        self.reset()
    
    def reset(self):
        self.last_error = None
        self.integral_error = 0
    
    def update(self, error, dt):
        # Derivative
        if self.last_error is None:
            self.last_error = error
            return 0.0
        derivative_error = (error - self.last_error) / dt
        self.last_error = error
        
        # Integral
        self.integral_error += error * dt
        
        # Output
        output = self.Kp * error + self.Ki * self.integral_error + self.Kd * derivative_error
        if self.min_output is not None:
            output = max(output, self.min_output)
        if self.max_output is not None:
            output = min(output, self.max_output)
        return output


class TemporaryCommander(Node):
    def __init__(self):
        super().__init__('temporary_commander')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.pantil_pub = self.create_publisher(PanTiltCmdDeg, '/pan_tilt_cmd_deg', 10)
        self.pan_tilt_pitch = 15.0
        self.pan_tilt_yaw = 0.0
        
        self.aruco_pixel_sub = self.create_subscription(Int32MultiArray, '/aruco_pixel', self.aruco_pixel_callback, 10)
        self.aruco_pixel_x, self.aruco_pixel_y = 1280 // 2, 720 // 2
        
        self.amcl_pose_sub = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.amcl_pose_callback, 10)
        self.amcl_pose = None
        
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, QoSProfile(depth = 10, reliability = ReliabilityPolicy.BEST_EFFORT))
        self.odom = None
        
        self.block_link_pub = self.create_publisher(PoseStamped, '/block_link', 10)
        self.target_link_pub = self.create_publisher(PoseStamped, '/target_link', 10)
    
    def warp_angle(self, angle):
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
    
    def quaternion_to_yaw(self, quaternion):
        q_0, q_1, q_2, q_3 = quaternion.w, quaternion.x, quaternion.y, quaternion.z
        return math.atan2(2 * (q_0 * q_3 + q_1 * q_2), 1 - 2 * (q_2 * q_2 + q_3 * q_3))
    
    def aruco_pixel_callback(self, msg):
        self.aruco_pixel_x, self.aruco_pixel_y = msg.data[0], msg.data[1]
    
    def amcl_pose_callback(self, msg):
        self.amcl_pose = msg
    
    def odom_callback(self, msg):
        self.odom = msg
    
    def get_current_yaw(self):
        if self.amcl_pose is None:
            return None
        return self.warp_angle(self.quaternion_to_yaw(self.amcl_pose.pose.pose.orientation))
    
    def pan_tilt(self, pitch, yaw, speed):
        pan_tilt_deg_cmd = PanTiltCmdDeg()
        pan_tilt_deg_cmd.pitch = pitch
        pan_tilt_deg_cmd.yaw = yaw
        pan_tilt_deg_cmd.speed = speed
        self.pantil_pub.publish(pan_tilt_deg_cmd)
    
    def cmd_vel(self, linear_x = 0, angular_z = 0):
        cmd = Twist()
        cmd.linear.x = linear_x
        cmd.angular.z = angular_z
        self.cmd_vel_pub.publish(cmd)
    
    def update_pan_tilt(self, pitch = 15.0, yaw = 0.0, speed = 20):
        self.pan_tilt_pitch = pitch
        self.pan_tilt_yaw = yaw
        self.pan_tilt(pitch = self.pan_tilt_pitch, yaw = self.pan_tilt_yaw, speed = speed)
        time.sleep(1.0) # TODO: remove
    
    def stop(self):
        self.cmd_vel()
    
    def turn_to_yaw(self, yaw, error_threshold = 0.1, command_threshold = 0.1):
        while self.amcl_pose is None:
            rclpy.spin_once(self)
        
        controller = PIDController(0.52, 0.03, 0.26, min_output = -0.7, max_output =0.7)
        last_time = self.get_clock().now().to_msg().nanosec / 1e9
        cmd = Twist()
        
        while True:
            current_time = self.get_clock().now().to_msg().nanosec / 1e9
            dt = current_time - last_time
            last_time = current_time
            
            yaw_error = self.warp_angle(yaw - self.get_current_yaw())
            # self.get_logger().info(f'yaw: {np.rad2deg(self.get_current_yaw())}, yaw_target: {np.rad2deg(yaw)}, yaw_error: {np.rad2deg(yaw_error)}')
            omega = controller.update(yaw_error, dt)
            self.cmd_vel(angular_z = omega)
            
            rclpy.spin_once(self, timeout_sec = 0.02)
            
            if abs(yaw_error) <= error_threshold and abs(omega) <= command_threshold:
                self.stop()
                break


class TemporaryClient(Node):
    def __init__(self):
        super().__init__('temporary_client')
        
        self.grasp_action_client = self.create_client(GraspAction, 'grasp_action')
        self.grasp_action_client.wait_for_service()
        
        self.grasp_query_solved_client = self.create_client(GraspQuery, 'grasp_is_solved')
        self.grasp_query_solved_client.wait_for_service()
        
        self.grasp_query_holding_client = self.create_client(GraspQuery, 'grasp_is_holding')
        self.grasp_query_holding_client.wait_for_service()
        
        self.block_center_pose_sub = self.create_subscription(PoseStamped, '/block_center_pose', self.block_center_pose_callback, 10)
        self.block_center_pose = None
        self.block_center_pose_expire_time = 1.5
    
    def block_center_pose_callback(self, msg):
        self.block_center_pose = msg
    
    def get_block_center_pose(self, timeout_sec = 0.1):
        rclpy.spin_once(self, timeout_sec = timeout_sec)
        if self.block_center_pose is None or self.block_center_pose.header.stamp.sec + self.block_center_pose_expire_time < self.get_clock().now().to_msg().sec:
            return None
        return self.block_center_pose

    def grasp_action(self, action):
        request = GraspAction.Request()
        request.action = action
        future = self.grasp_action_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.result
    
    def grasp_query_solved(self):
        request = GraspQuery.Request()
        future = self.grasp_query_solved_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.result
    
    def grasp_query_holding(self):
        request = GraspQuery.Request()
        future = self.grasp_query_holding_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        response = future.result()
        return response.result


class MyNavigator(BasicNavigator):
    def __init__(self):
        super().__init__()

    def initialize(self, initial_pose):
        self.setInitialPose(initial_pose)
        # self.lifecycleStartup()
        self.waitUntilNav2Active()
        self.clearAllCostmaps()
    
    def shutdown(self):
        # navigator.lifecycleShutdown()
        exit(0)

    def go_to_goal(self, goal: PoseStamped, blocking = True):
        self.goToPose(goal)
        if blocking:
            while not self.isTaskComplete():
                feedback = self.getFeedback()
            return self.getResult()
        else:
            return None


def main():
    rclpy.init()
    
    # Wait for tf to be ready
    tf_reader = TF_Reader()
    while tf_reader.get_arm_to_base() is None:
        rclpy.spin_once(tf_reader)
    T_b0 = transform_to_matrix(tf_reader.get_arm_to_base().transform)
    
    # Nodes
    navigator = MyNavigator()
    temporary_client = TemporaryClient()
    temporary_commander = TemporaryCommander()
    
    # Points
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.3025282477783394
    initial_pose.pose.position.y = -0.36741189949949404
    initial_pose.pose.orientation.z = -0.7140501663813629
    initial_pose.pose.orientation.w = 0.7000945363954414

    goal_A = PoseStamped()
    goal_A.header.frame_id = 'map'
    goal_A.header.stamp = navigator.get_clock().now().to_msg()
    goal_A.pose.position.x = 0.2266274822848437
    goal_A.pose.position.y = -3.4258465986026695
    goal_A.pose.orientation.z = -0.6686671777907486
    goal_A.pose.orientation.w = 0.7435618369344646
    
    goal_B_frontier = PoseStamped()
    goal_B_frontier.header.frame_id = 'map'
    goal_B_frontier.header.stamp = navigator.get_clock().now().to_msg()
    goal_B_frontier.pose.position.x = 0.2266274822848437
    goal_B_frontier.pose.position.y = -2.7258465986026695
    goal_B_frontier.pose.orientation.z = -0.6686671777907486
    goal_B_frontier.pose.orientation.w = 0.7435618369344646
    
    goal_B = PoseStamped()
    goal_B.header.frame_id = 'map'
    goal_B.header.stamp = navigator.get_clock().now().to_msg()
    goal_B.pose.position.x = 2.8567487875336247
    goal_B.pose.position.y = -3.320366191076802
    goal_B.pose.orientation.z = -0.007822402212393793
    goal_B.pose.orientation.w = 0.9999694045437728
    
    goal_S = PoseStamped()
    goal_S.header.frame_id = 'map'
    goal_S.header.stamp = navigator.get_clock().now().to_msg()
    goal_S.pose.position.x = 0.3025282477783394 # 0.3725282477783394
    goal_S.pose.position.y = -0.35741189949949404 # -0.45741189949949404
    goal_S.pose.orientation.z = 0.924 # -0.7000945363954414
    goal_S.pose.orientation.w = 0.383 # -0.7140501663813629
    
    # Initialization
    navigator.initialize(initial_pose)
    
    # Go to A
    navigator.go_to_goal(goal_A, blocking = False)
    
    # - Check aruco insight
    insight = False
    while not navigator.isTaskComplete():
        block_pose = temporary_client.get_block_center_pose()
        if block_pose is not None:
            d = math.sqrt(block_pose.pose.position.x ** 2 + block_pose.pose.position.y ** 2)
            temporary_client.get_logger().info(f'aruco detected {d} meters away.')
            if d < 1.2:
                insight = True
                navigator.cancelTask()
                break
            else:
                continue
        temporary_client.get_logger().info(f'checking aruco insight ...')
    
    # Approach to block
    if insight:
        ctrl_linear = PIDController(0.5, 0.01, 0.1, min_output = 0.0, max_output = 0.2)
        ctrl_angular = PIDController(0.5, 0.01, 0.1, min_output = -0.5, max_output = 0.5)
        last_time = temporary_commander.get_clock().now().to_msg().nanosec / 1e9
        while True:
            temporary_commander.get_logger().info(f'approaching ...')
            current_time = temporary_commander.get_clock().now().to_msg().nanosec / 1e9
            dt = current_time - last_time
            last_time = current_time
            
            block_pose = temporary_client.get_block_center_pose()
            if block_pose is not None:
                T_0a = pose_to_matrix(block_pose.pose)
        
    else:
        pass # TODO: look around
    
    """
    # - Approach to block
    if insight:
        navigator.cancelTask()
        
        # Go to B_frontier
        navigator.go_to_goal(goal_B_frontier)
        
        # Approaching
        ctrl_linear = PIDController(0.5, 0.01, 0.1, min_output = 0.0, max_output = 0.2)
        ctrl_angular = PIDController(0.5, 0.01, 0.1, min_output = -0.5, max_output = 0.5)
        last_time = temporary_commander.get_clock().now().to_msg().nanosec / 1e9
        while True:
            temporary_commander.get_logger().info(f'approaching ...')
            current_time = temporary_commander.get_clock().now().to_msg().nanosec / 1e9
            dt = current_time - last_time
            last_time = current_time
            
            block_pose = temporary_client.get_block_center_pose()
            if block_pose is not None:
                T_0a = pose_to_matrix(block_pose.pose)
                T_ba = np.dot(T_b0, T_0a)
                dx, dy = T_ba[0, 3], T_ba[1, 3]
                
                x = T_ba[:3, 0]
                y = T_ba[:3, 1]
                z = T_ba[:3, 2]
                dir = np.array([dx, dy, 0])
                dotmax_axis = x
                if np.linalg.norm(dir * y) > np.linalg.norm(dir * dotmax_axis):
                    dotmax_axis = y
                if np.linalg.norm(dir * z) > np.linalg.norm(dir * dotmax_axis):
                    dotmax_axis = z
                
                dotmax_axis /= np.linalg.norm(dotmax_axis) # normalize, TODO: remove?
                dist_base_to_block = 0.40
                dx += dotmax_axis[0] * dist_base_to_block
                dy += dotmax_axis[1] * dist_base_to_block
                
                d = math.sqrt(dx ** 2 + dy ** 2)
                delta_theta = math.atan2(dy, dx)
                
                v = ctrl_linear.update(d, dt)
                omega = ctrl_angular.update(delta_theta, dt)
                
                temporary_commander.get_logger().info(f'd: {d}, delta_theta: {delta_theta}')
                temporary_commander.get_logger().info(f'dx: {dx}, dy: {dy}')
                temporary_commander.get_logger().info(f'v: {v}, omega: {omega}')
                temporary_commander.get_logger().info(f'x: {x}, y: {y}, z: {z}')
                temporary_commander.get_logger().info(f'dotmax_axis: {dotmax_axis}')
                temporary_commander.get_logger().info(f'T_ba: {T_ba}')
                
                block_link_pose = PoseStamped()
                block_link_pose.header.stamp = temporary_commander.get_clock().now().to_msg()
                block_link_pose.header.frame_id = 'base_link'
                block_link_pose.pose = matrix_to_pose(T_ba)
                temporary_commander.block_link_pub.publish(block_link_pose)
                
                target_link_pose = PoseStamped()
                target_link_pose.header.stamp = temporary_commander.get_clock().now().to_msg()
                target_link_pose.header.frame_id = 'base_link'
                target_link_pose.pose.position.x = dx
                target_link_pose.pose.position.y = dy
                target_link_pose.pose.orientation.w = 1.0
                temporary_commander.target_link_pub.publish(target_link_pose)
                
                if d < 0.05:
                    v = 0.0
                if delta_theta < 0.05:
                    omega = 0.0
                if d < 0.05 and delta_theta < 0.05:
                    break
                
                temporary_commander.cmd_vel(linear_x = v, angular_z = omega)
                
                # turn pan_tilt
                # if temporary_commander.aruco_pixel_x < 1280 // 4 or temporary_commander.aruco_pixel_x > 1280 // 4 * 3:
                #     yaw = (1280 // 2 - temporary_commander.aruco_pixel_x) / 1280 * 69.4
                #     yaw = np.clip(yaw, -30, 30)
                #     temporary_commander.update_pan_tilt(yaw = yaw)
                
                rclpy.spin_once(temporary_commander, timeout_sec = 0.02)
            else:
                pass # TODO: do not see?
    else:
        pass # TODO: look around
    
    while not temporary_client.grasp_query_solved():
        temporary_commander.cmd_vel(linear_x = 0.1)
    temporary_commander.stop()
    
    # Recover the pan_tilt
    # temporary_commander.update_pan_tilt(yaw = 0.0)
    # time.sleep(2.0)
    """
    
    # Grasp
    temporary_client.grasp_action('grasp')
    while not temporary_client.grasp_query_holding():
        time.sleep(0.5)
    temporary_client.get_logger().info(f'held.')
    
    # Check, TODO: Judge whether the grasp is successful (by checking the aruco insight) -> regrasp

    # Go to B
    temporary_commander.turn_to_yaw(np.deg2rad(0))
    navigator.go_to_goal(goal_B)
    
    # Release, TODO: in place
    temporary_client.grasp_action('release')
    
    # Go to S
    temporary_commander.turn_to_yaw(np.deg2rad(135))
    navigator.go_to_goal(goal_S)

    # Shutdown
    navigator.shutdown()


if __name__ == '__main__':
    main()

