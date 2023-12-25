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

from grasp_interfaces.srv import GraspAction, GraspQuery


class ArmController(Node):
    def __init__(self, tf_reader):
        # [ arm controller ]
        super().__init__('ArmController')
        self.tf_reader = tf_reader

        self.cmd_pub = self.create_publisher(JointSingleCommand, '/px100/commands/joint_single', 10)
        self.group_pub = self.create_publisher(JointGroupCommand, '/px100/commands/joint_group', 10)
        self.pantil_pub = self.create_publisher(PanTiltCmdDeg, '/pan_tilt_cmd_deg', 10)
        self.fb_sub = self.create_subscription(JointState, '/joint_states', self.joint_states_callback, 10)
        
        self.timer = self.create_timer(0.2, self.fsm_timer_callback)

        self.robot_des: mrd.ModernRoboticsDescription = getattr(mrd, 'px100')
        self.arm_command = JointSingleCommand()
        self.arm_group_command = JointGroupCommand()
        
        self.cnt = 0
        self.thred = 0.12
        self.joint_pos = []
        self.num_joints = 4
        
        self.joint_lower_limits = [-1.5, -0.4, -1.6, -1.8]
        self.joint_upper_limits = [1.5, 0.9, 1.7, 1.8]
        
        self.initial_guesses = [[0.0] * self.num_joints] * 3
        self.initial_guesses[1][0] = np.deg2rad(-30)
        self.initial_guesses[2][0] = np.deg2rad(30)

        self.shoulder_offset = -0.1 # gravity compensation

        # [ application ]
        self.arm_target_pose_pub = self.create_publisher(PoseStamped, '/arm_target_pose', 10)
        self.aruco_target_pose_pub = self.create_publisher(PoseStamped, '/aruco_target_pose', 10)
        self.block_center_pose_pub = self.create_publisher(PoseStamped, '/block_center_pose', 10)
        self.aruco_poses_sub = self.create_subscription(PoseArray, '/aruco_poses', self.aruco_poses_callback, 10)
        
        self.block_length = 0.035
        self.aruco_expired_time = 1.0
        self.aruco_poses = None
        self.machine_state = 'INIT'
        
        self.grasp_action_srv = self.create_service(GraspAction, 'grasp_action', self.grasp_action_callback)
        self.grasp_is_solved_srv = self.create_service(GraspQuery, 'grasp_is_solved', self.grasp_is_solved_callback)
        self.grasp_is_holding_srv = self.create_service(GraspQuery, 'grasp_is_holding', self.grasp_is_holding_callback)
        self.is_solved = False
        self.allow_execute_trigger = False
    
    def grasp_action_callback(self, request, response): # reset, grasp, release
        if request.action == 'reset':
            self.machine_state = 'INIT'
        elif request.action == 'grasp': # must be called after 'reset'
            self.allow_execute_trigger = True
        elif request.action == 'release':
            self.machine_state = 'RELEASE'
        
        response.result = True
        self.get_logger().info(f'grasp_action({request.action}) called -> {response.result}')
        return response

    def grasp_is_solved_callback(self, request, response):
        response.result = self.is_solved if self.is_solved is not None else False
        self.is_solved = None
        self.get_logger().info(f'grasp_is_solved() called -> {response.result}')
        return response

    def grasp_is_holding_callback(self, request, response):
        response.result = (self.machine_state == 'HOLD')
        self.get_logger().info(f'grasp_is_holding() called -> {response.result}')
        return response

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
                    self.get_logger().info('Single joint moving...')
                    return False
            elif name == 'gripper':
                return True
            else:
                self.get_logger().info('Unexpect name!')
                return False

    def set_group_pos(self, pos_list):
        if len(pos_list) != self.num_joints:
            self.get_logger().info('Unexpect length of list!')
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
            self.get_logger().info('No joint states.')
            return False
    
    def kinematics(self, joint_state):
        return mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_state)

    def go_home(self):
        return self.set_group_pos([1.5, self.shoulder_offset, 1.5, -1.5])

    def go_frontier(self):
        return self.set_group_pos([0, self.shoulder_offset, 1.5, -1.5])
    
    def go_handup(self):
        return self.set_group_pos([1.5, self.shoulder_offset, -1.4, 0.0])

    def matrix_control(self, T_sd, custom_guess: list[float] = None, execute: bool = True, custom_joints = None, waist_offset = 0.0, delay = 1.0):
        initial_guesses = self.initial_guesses if custom_guess is None else [custom_guess]
        for guess in initial_guesses:
            theta_list, solution_found = mr.IKinSpace(
                Slist = self.robot_des.Slist,
                M = self.robot_des.M,
                T = T_sd,
                thetalist0 = guess,
                eomg = 0.02,
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
                # TODO BEGIN
                # if joint_list[0] > 0:
                #     joint_list[0] *= 1.09
                joint_list[0] += 0.06 # 0.075 # 0.015
                # TODO END
                
                # waist offset
                joint_list[0] += waist_offset
                
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

    def pan_tilt(self, pitch, yaw, speed):
        pan_tilt_deg_cmd = PanTiltCmdDeg()
        pan_tilt_deg_cmd.pitch = pitch
        pan_tilt_deg_cmd.yaw = yaw
        pan_tilt_deg_cmd.speed = speed
        self.pantil_pub.publish(pan_tilt_deg_cmd)

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
        
        def radius_oriented_offset(matrix, forward, upward):
            dx, dy = matrix[0, 3], matrix[1, 3]
            d = math.sqrt(dx * dx + dy * dy)
            return np.array([
                [matrix[0, 0], matrix[0, 1], matrix[0, 2], matrix[0, 3] + dx * forward / d],
                [matrix[1, 0], matrix[1, 1], matrix[1, 2], matrix[1, 3] + dy * forward / d],
                [matrix[2, 0], matrix[2, 1], matrix[2, 2],           matrix[2, 3] + upward],
                [           0,            0,            0,                               1]
            ])
        
        def substitute_R(matrix):
            dx, dy, dz = matrix[0, 3], matrix[1, 3], matrix[2, 3]
            d = math.sqrt(dx * dx + dy * dy)
            nx, ny = dx / d, dy / d
            return np.array([
                [nx, -ny, 0, dx],
                [ny,  nx, 0, dy],
                [ 0,   0, 1, dz],
                [ 0,   0, 0,  1]
            ])
        
        def compute_yaw_inclination(T_block_center):
            dx, dy = T_block_center[0, 3], T_block_center[1, 3]
            theta = math.atan2(dy, dx)
            
            x = T_block_center[:3, 0]
            y = T_block_center[:3, 1]
            z = T_block_center[:3, 2]
            zmin_axis = x
            if abs(y[2]) < abs(zmin_axis[2]):
                zmin_axis = y
            if abs(z[2]) < abs(zmin_axis[2]):
                zmin_axis = z
            beta = math.atan2(zmin_axis[1], zmin_axis[0])
            
            yaw_relative = beta - theta
            yaw_regular = (yaw_relative - np.pi / 4) % (np.pi / 2) - np.pi / 4
            return yaw_regular
        
        if len(self.joint_pos) == 7:
            if self.machine_state == 'INIT':
                self.get_logger().info('Initializing pan tilt and arm ...')
                self.pan_tilt(pitch = 15.0, yaw = 0.0, speed = 20)
                if self.go_home() and self.gripper(1.5, 0.5):
                    self.get_logger().info('Ready.')
                    self.machine_state = 'DETECT'
                    
                    self.valid_times = 0
                    self.is_solved = False
                    self.allow_execute_trigger = False
                    
                    time.sleep(1.0)
            elif self.machine_state == 'DETECT':
                current_aruco_pose = self.get_active_aruco_pose()
                if current_aruco_pose:
                    # self.get_logger().info('Aruco detected.')
                    
                    # Compute T_0a
                    # TODO: 每次刚 bringup 后的第一次运行会出现 action_matrix 高出正确位置一截的情况（猜测是 T_0c 的问题，没具体检查）
                    T_0c = compute_T_0c()
                    T_ca = compute_T_ca()
                    T_0a = np.dot(T_0c, T_ca)
                    R_0a = T_0a[:3, :3]
                    t_0a = T_0a[:3, 3]
                    
                    # Inclination of block within -pi/4 ~ pi/4
                    self.inc_correct_factor = -0.025 / (np.pi / 4)
                    self.inc_yaw = compute_yaw_inclination(T_0a)
                    
                    # Correct (ahead_)action_matrix translation and compute valid action_matrix orientation
                    self.action_matrix = T_0a
                    self.action_matrix = radius_oriented_offset(self.action_matrix, -0.04, 0.005)
                    self.action_matrix = substitute_R(self.action_matrix)
                    self.ahead_action_matrix = T_0a
                    self.ahead_action_matrix = radius_oriented_offset(self.ahead_action_matrix, 0.00, 0.005) # radius_oriented_offset(self.ahead_action_matrix, 0.02, 0.005)
                    self.ahead_action_matrix = substitute_R(self.ahead_action_matrix)
                    
                    # Publish selected aruco pose
                    aruco_target_pose = PoseStamped()
                    aruco_target_pose.header.stamp = self.get_clock().now().to_msg()
                    aruco_target_pose.header.frame_id = 'camera_color_optical_frame'
                    aruco_target_pose.pose = current_aruco_pose
                    self.aruco_target_pose_pub.publish(aruco_target_pose)
                    
                    # Publish block center pose
                    block_center_pose = PoseStamped()
                    block_center_pose.header.stamp = self.get_clock().now().to_msg()
                    block_center_pose.header.frame_id = 'px100/base_link'
                    block_center_pose.pose = self.matrix_to_pose(T_0a)
                    self.block_center_pose_pub.publish(block_center_pose)
                    
                    # Publish arm target pose
                    arm_target_pose = PoseStamped()
                    arm_target_pose.header.stamp = self.get_clock().now().to_msg()
                    arm_target_pose.header.frame_id = 'px100/base_link'
                    arm_target_pose.pose = self.matrix_to_pose(self.action_matrix)
                    self.arm_target_pose_pub.publish(arm_target_pose)
                    
                    # Test solution
                    _, solution_found_frontier, solution_valid_frontier, _ = self.matrix_control(self.action_matrix, execute = False)
                    _, solution_found_ahead, solution_valid_ahead, _ = self.matrix_control(self.ahead_action_matrix, execute = False)
                    self.is_solved = solution_valid_frontier and solution_valid_ahead
                    # self.get_logger().info(f'[Frontier] Solution found: {solution_found_frontier}; solution valid: {solution_valid_frontier}.')
                    # self.get_logger().info(f'[Ahead] Solution found: {solution_found_ahead}; solution valid: {solution_valid_ahead}.')
                    
                    # Execute solution or block
                    if self.is_solved and self.allow_execute_trigger:
                        self.valid_times += 1
                        if self.valid_times > 3:
                            self.valid_times = 0
                            self.allow_execute_trigger = False
                            self.machine_state = 'TWIST_WAIST'
            elif self.machine_state == 'TWIST_WAIST':
                self.get_logger().info('Twisting waist ...')
                self.gripper(1.5, 1.0)
                _, _, _, reached = self.matrix_control(
                    self.action_matrix,
                    custom_joints = [None, self.shoulder_offset, -1.4, 0.0],
                    waist_offset = self.inc_yaw * self.inc_correct_factor,
                    delay = 0.5
                )
                if reached:
                    self.get_logger().info('Done.')
                    self.machine_state = 'GO_FRONTIER'
            elif self.machine_state == 'GO_FRONTIER':
                self.get_logger().info('Going frontier ...')
                self.get_logger().info(f'inc_yaw: {self.inc_yaw}, offset: {self.inc_yaw * self.inc_correct_factor}')
                _, _, _, reached = self.matrix_control(self.action_matrix, waist_offset = self.inc_yaw * self.inc_correct_factor, delay = 1.0)
                if reached:
                    self.get_logger().info('done.')
                    self.machine_state = 'GO_AHEAD'
            elif self.machine_state == 'GO_AHEAD':
                self.get_logger().info('Going ahead ...')
                self.get_logger().info(f'inc_yaw: {self.inc_yaw}, offset: {self.inc_yaw * self.inc_correct_factor}')
                _, _, _, reached = self.matrix_control(self.ahead_action_matrix, waist_offset = self.inc_yaw * self.inc_correct_factor, delay = 1.0)
                if reached:
                    self.get_logger().info('done.')
                    self.gripper(0.7, 1.0)
                    self.machine_state = 'HAND_UP'
            elif self.machine_state == 'HAND_UP':
                self.get_logger().info('Handing up ...')
                if self.go_handup():
                    self.get_logger().info('done.')
                    self.machine_state = 'HOLD'
            elif self.machine_state == 'HOLD':
                self.get_logger().info('Holding ...')
                if self.go_home():
                    self.get_logger().info('done.')
            elif self.machine_state == 'RELEASE':
                self.machine_state = 'INIT' # TODO
            else:
                self.get_logger().info('Invalid machine state.')

