import time
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Pose, PoseStamped

from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
from pan_tilt_msgs.msg import PanTiltCmdDeg

import modern_robotics as mr
import tf_transformations
import numpy as np


class ArmController(Node):
    def __init__(self, tf_reader):
        super().__init__('ArmController')
        self.tf_reader = tf_reader
        
        self.cmd_pub = self.create_publisher(JointSingleCommand, '/px100/commands/joint_single', 10)
        self.group_pub = self.create_publisher(JointGroupCommand, '/px100/commands/joint_group', 10)
        self.pantil_pub = self.create_publisher(PanTiltCmdDeg, '/pan_tilt_cmd_deg', 10)
        self.fb_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        self.pub_timer = self.create_timer(0.2, self.fsm_timer_callback)

        self.robot_des: mrd.ModernRoboticsDescription = getattr(mrd, 'px100')
        self.arm_command = JointSingleCommand()
        self.arm_group_command = JointGroupCommand()
        
        self.cnt = 0
        self.thred = 0.1
        self.joint_pos = []
        self.moving_time = 2.0
        self.num_joints = 4
        
        self.joint_lower_limits = [-1.5, -0.4, -1.6, -1.8]
        self.joint_upper_limits = [1.5, 0.9, 1.7, 1.8]
        
        self.initial_guesses = [[0.0] * self.num_joints] * 3
        self.initial_guesses[1][0] = np.deg2rad(-30)
        self.initial_guesses[2][0] = np.deg2rad(30)

        self.shoulder_offset = -0.05 # gravity compensation

        # === application ===
        self.arm_target_pose_pub = self.create_publisher(PoseStamped, '/arm_target_pose', 10)
        self.aruco_target_pose_pub = self.create_publisher(PoseStamped, '/aruco_target_pose', 10)
        self.block_center_pose_pub = self.create_publisher(PoseStamped, '/block_center_pose', 10)
        self.aruco_poses_sub = self.create_subscription(PoseArray, '/aruco_poses', self.aruco_poses_callback, 10)
        
        self.block_length = 0.04
        self.aruco_expired_time = 1.0
        # service
        self.aruco_poses = None
        self.machine_state = 'INIT'
    
    def joint_states_callback(self, msg):
        if len(msg.name) == 7:
            self.joint_pos.clear()
            for i in range(7):
                self.joint_pos.append(msg.position[i])
    
    def aruco_poses_callback(self, msg):
        self.aruco_poses = msg
    
    def get_active_aruco_pose(self): # get the first aruco pose (within expired time)
        if self.aruco_poses:
            if len(self.aruco_poses.poses) > 0:
                sec, _ = self.get_clock().now().seconds_nanoseconds()
                if sec - self.aruco_poses.header.stamp.sec > self.aruco_expired_time:
                    return None
                return self.aruco_poses.poses[0]
        return None
    
    def matrix_to_pose(self, M):
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = M[0, 3], M[1, 3], M[2, 3]
        quat = tf_transformations.quaternion_from_matrix(M)
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat[0], quat[1], quat[2], quat[3]
        return pose

    def set_single_pos(self, name, pos):
        self.arm_command.name = name
        self.arm_command.cmd = pos
        self.cmd_pub.publish(self.arm_command)
        
        joint_idx = {'waist': 0, 'shoulder': 1, 'elbow': 2, 'wrist_angle': 3}
        if len(self.joint_pos) == 7:
            if name in joint_idx.keys():
                dis = np.abs(pos - self.joint_pos[joint_idx[name]])
                if dis < self.thred:
                    return True
                else:
                    self.get_logger().info('single joint moving...')
                    return False
            elif name == 'gripper':
                return True
            else:
                self.get_logger().info('unexpect name!')
                return False

    def set_group_pos(self, pos_list):
        if len(pos_list) != self.num_joints:
            self.get_logger().info('unexpect length of list!')
            return False
        
        for i in range(self.num_joints):
            pos_list[i] = math.fmod(pos_list[i], 2 * math.pi)
            pos_list[i] = pos_list[i] - 2 * math.pi if pos_list[i] > math.pi else pos_list[i]
            pos_list[i] = pos_list[i] + 2 * math.pi if pos_list[i] < -math.pi else pos_list[i]
        # pos_list = self._wrap_theta_list(np.array(pos_list))
        
        self.arm_group_command.name = 'arm'
        self.arm_group_command.cmd = pos_list
        self.group_pub.publish(self.arm_group_command)
        
        if len(self.joint_pos) == 7:
            thred = self.thred
            check_pos = self.joint_pos
            return np.abs(pos_list[0] - check_pos[0]) < thred and np.abs(pos_list[1] - check_pos[1]) < thred and np.abs(pos_list[2] - check_pos[2]) < thred and np.abs(pos_list[3] - check_pos[3]) < thred
        else:
            self.get_logger().info('no joint states.')
            return False
    
    def kinematics(self, joint_state):
        return mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_state)

    def go_home(self):
        state = self.set_group_pos([1.57, self.shoulder_offset, 1.5, -1.5])
        return state
    
    def go_hold(self):
        state = self.set_group_pos([0.0, self.shoulder_offset, -1.5, 0.0])
        time.sleep(0.2)
        state = self.set_group_pos([0.0, self.shoulder_offset, -1.4, 0.0])
        return state

    def matrix_control(self, T_sd, custom_guess: list[float] = None, execute: bool = True, custom_joints = None, delay = 1.0):
        initial_guesses = self.initial_guesses if custom_guess is None else [custom_guess]
        for guess in initial_guesses:
            theta_list, solution_found = mr.IKinSpace(
                Slist = self.robot_des.Slist,
                M = self.robot_des.M,
                T = T_sd,
                thetalist0 = guess,
                eomg = 0.1,
                ev = 0.002,
            )
            solution_valid = False
            if solution_found:
                theta_list = self._wrap_theta_list(theta_list)
                solution_valid = self._check_joint_limits(theta_list)
            
            reached = False
            if solution_valid and execute:
                # target joints
                joint_list = [theta_list[0], theta_list[1] + self.shoulder_offset, theta_list[2], theta_list[3]]
                                                    # TODO BEGIN: 直接写死
                if joint_list[0] > 0:
                    joint_list[0] *= 1.09
                joint_list[0] -= 0.015
                                                    # TODO END
                
                # substitute custom joints
                if (custom_joints is not None) and (len(custom_joints) == self.num_joints):
                    for i in range(self.num_joints):
                        if custom_joints[i] is not None:
                            joint_list[i] = custom_joints[i]
                
                reached = self.set_group_pos(joint_list)
                time.sleep(delay)
            return theta_list, solution_found, solution_valid, reached
        
        # no solution found
        return None, False, False

    def gripper(self, effort, delay: float):
        if len(self.joint_pos) == 7:
            gripper_state = self.set_single_pos('gripper', float(effort))
            time.sleep(delay)
            return gripper_state

    def _wrap_theta_list(self, theta_list: list[np.ndarray]) -> list[np.ndarray]:
        # to [-pi, pi], TODO: not verified
        REV = 2 * np.pi
        theta_list = (theta_list + np.pi) % REV - np.pi
        for x in range(len(theta_list)):
            if round(theta_list[x], 3) < round(self.joint_lower_limits[x], 3):
                theta_list[x] += REV
            elif round(theta_list[x], 3) > round(self.joint_upper_limits[x], 3):
                theta_list[x] -= REV
        return theta_list

    def _check_joint_limits(self, theta_list) -> bool:
        if len(theta_list) != self.num_joints:
            self.get_logger().info('unexpect length of list!')
            return False
        for i in range(4):
            if theta_list[i] > self.joint_upper_limits[i] or theta_list[i] < self.joint_lower_limits[i]:
                self.get_logger().info(f'joint {i} out of range: {theta_list[i]}')
                return False
        return True
    
    def fsm_timer_callback(self):
        def compute_T_0c():
            trans = self.tf_reader.get_arm_to_cam().transform
            translation = np.array([trans.translation.x, trans.translation.y, trans.translation.z])
            rotation = [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]
            R = tf_transformations.quaternion_matrix(rotation)
            matrix = np.eye(4)
            matrix[:3, :3] = R[:3, :3]
            matrix[:3, 3] = translation
            return np.linalg.inv(matrix)
        
        def compute_T_ca():
            quat_ca = [current_aruco_pose.orientation.x, current_aruco_pose.orientation.y, current_aruco_pose.orientation.z, current_aruco_pose.orientation.w]
            R_ca = tf_transformations.quaternion_matrix(quat_ca)
            t_ca = np.array([current_aruco_pose.position.x, current_aruco_pose.position.y, current_aruco_pose.position.z])
            T_ca = np.eye(4)
            T_ca[:3, :3] = R_ca[:3, :3]
            T_ca[:3, 3] = t_ca
            T_ca[0 : 3, 3] -= self.block_length / 2.0 * T_ca[0 : 3, 2] # Correct T_ca translation (from surface center to body center)
            return T_ca
        
        if len(self.joint_pos) == 7:
            if self.machine_state == 'INIT':
                self.get_logger().info('initializing ...')
                pan_tilt_deg_cmd = PanTiltCmdDeg()
                pan_tilt_deg_cmd.pitch = 15.0
                pan_tilt_deg_cmd.yaw = 0.0
                pan_tilt_deg_cmd.speed = 10
                self.pantil_pub.publish(pan_tilt_deg_cmd)
                
                if self.go_home() and self.gripper(1.5, 0.5):
                    self.get_logger().info('done.')
                    self.machine_state = 'DETECT'
                    self.valid_times = 0
                    time.sleep(1.0)
            elif self.machine_state == 'DETECT':
                current_aruco_pose = self.get_active_aruco_pose()
                if current_aruco_pose:
                    self.get_logger().info('aruco detected.')
                    
                    # Compute T_0a
                    T_0c = compute_T_0c()
                    T_ca = compute_T_ca()
                    T_0a = np.dot(T_0c, T_ca)
                    
                    # Correct T_0a translation
                    dx, dy = T_0a[0, 3], T_0a[1, 3]
                    d = math.sqrt(dx * dx + dy * dy)
                    T_0a[0, 3] -= dx * 0.02 / d
                    T_0a[1, 3] -= dy * 0.02 / d
                    T_0a[2, 3] += 0.005
                    
                    # Compute valid T_0a orientation
                    dx, dy = T_0a[0, 3], T_0a[1, 3]
                    d = math.sqrt(dx * dx + dy * dy)
                    nx, ny = dx / d, dy / d
                    T_0a[:3, :3] = np.array([
                        [nx, -ny, 0],
                        [ny,  nx, 0],
                        [ 0,   0, 1]
                    ])
                    self.action_matrix = T_0a # TODO: 每次刚 bringup 后的第一次运行会出现 T_0a 高出正确位置一截的情况（猜测是 T_0c 的问题，没具体检查）
                    
                    # Publish selected aruco pose
                    aruco_target_pose = PoseStamped()
                    aruco_target_pose.header.stamp = self.get_clock().now().to_msg()
                    aruco_target_pose.header.frame_id = 'camera_color_optical_frame'
                    aruco_target_pose.pose = current_aruco_pose
                    self.aruco_target_pose_pub.publish(aruco_target_pose)
                    
                    # Publish block center pose
                    block_center_pose = PoseStamped()
                    block_center_pose.header.stamp = self.get_clock().now().to_msg()
                    block_center_pose.header.frame_id = 'camera_color_optical_frame'
                    block_center_pose.pose = self.matrix_to_pose(T_ca)
                    self.block_center_pose_pub.publish(block_center_pose)
                    
                    # Publish arm target pose
                    arm_target_pose = PoseStamped()
                    arm_target_pose.header.stamp = self.get_clock().now().to_msg()
                    arm_target_pose.header.frame_id = 'px100/base_link'
                    arm_target_pose.pose = self.matrix_to_pose(self.action_matrix)
                    self.arm_target_pose_pub.publish(arm_target_pose)
                    
                    # Test solution
                    _, solution_found, solution_valid, reached = self.matrix_control(self.action_matrix, execute = False)
                    self.get_logger().info(f'solution found: {solution_found}; solution valid: {solution_valid}.')
                    if solution_valid:
                        self.valid_times += 1
                        if self.valid_times > 3:
                            self.valid_times = 0
                            self.machine_state = 'TWIST_WAIST'
            elif self.machine_state == 'TWIST_WAIST':
                self.get_logger().info('twisting waist ...')
                _, solution_found, solution_valid, reached = self.matrix_control(
                    self.action_matrix,
                    custom_joints = [None, self.shoulder_offset, -1.4, 0.0],
                    delay = 1.0
                )
                if reached:
                    self.get_logger().info('done.')
                    self.machine_state = 'GRASP'
            elif self.machine_state == 'GRASP':
                self.get_logger().info('grasping ...')
                _, solution_found, solution_valid, reached = self.matrix_control(self.action_matrix, delay = 1.0)
                if reached:
                    self.get_logger().info('done.')
                    self.gripper(0.7, 1.0)
                    self.machine_state = 'HAND_UP'
            elif self.machine_state == 'HAND_UP':
                self.get_logger().info('handing up ...')
                if self.go_hold():
                    self.get_logger().info('done.')
                    self.machine_state = 'HOLD'
            elif self.machine_state == 'HOLD':
                self.get_logger().info('holding ...')
                
            else:
                self.get_logger().info('unvalid machine state.')

