import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Pose, TransformStamped, PoseStamped

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations
from interbotix_xs_msgs.msg import JointSingleCommand, JointGroupCommand
import modern_robotics as mr
from interbotix_xs_modules.xs_robot import mr_descriptions as mrd
from pan_tilt_msgs.msg import PanTiltCmdDeg

import numpy as np
import time
import math
import array


class TF_Reader(Node):
    def __init__(self):
        super().__init__("tf_reader")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.read_timer = self.create_timer(0.8, self.timer_cb)
        self.trans = None
    
    def get_arm_to_cam(self):
        return self.trans
    
    def timer_cb(self):
        try:
            if not self.trans:
                self.get_logger().info("waiting for tf ...")
            now = rclpy.time.Time()
            self.trans = self.tf_buffer.lookup_transform("camera_color_optical_frame", "px100/base_link", now)
        except Exception:
            pass


class ArmController(Node):
    def __init__(self, tf_reader):
        super().__init__("ArmController")
        self.tf_reader = tf_reader
        
        self.cmd_pub = self.create_publisher(JointSingleCommand, "/px100/commands/joint_single", 10)
        self.group_pub = self.create_publisher(JointGroupCommand, "/px100/commands/joint_group", 10)
        self.pantil_pub = self.create_publisher(PanTiltCmdDeg, "/pan_tilt_cmd_deg", 10)
        self.fb_sub = self.create_subscription(JointState, "/joint_states", self.js_cb, 10)
        self.pub_timer = self.create_timer(0.2, self.work_cb)

        self.arm_command = JointSingleCommand()
        self.arm_group_command = JointGroupCommand()
        
        self.cnt = 0
        self.thred = 0.1
        self.joint_pos = []
        self.moving_time = 2.0
        self.num_joints = 4
        # self.joint_lower_limits = [-1.5, -0.4, -1.1, -1.4]
        # self.joint_upper_limits = [1.5, 0.9, 0.8, 1.8]
        self.joint_lower_limits = [-1.5, -0.4, -1.6, -1.8]
        self.joint_upper_limits = [1.5, 0.9, 1.7, 1.8]
        self.initial_guesses = [[0.0] * self.num_joints] * 3
        # self.initial_guesses[0][2] = np.deg2rad(80)
        # self.initial_guesses[0][3] = np.deg2rad(-80)
        self.initial_guesses[1][0] = np.deg2rad(-30)
        self.initial_guesses[2][0] = np.deg2rad(30)
        self.robot_des: mrd.ModernRoboticsDescription = getattr(mrd, 'px100')

        self.gripper_pressure: float = 0.5
        self.gripper_pressure_lower_limit: int = 150
        self.gripper_pressure_upper_limit: int = 350
        self.gripper_value = self.gripper_pressure_lower_limit + (self.gripper_pressure * (self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit))

        # custom
        self.shoulder_offset = -0.1
        self.block_length = 0.04
        
        self.arm_target_pose_pub = self.create_publisher(PoseStamped, "/arm_target_pose", 10)
        self.aruco_target_pose_pub = self.create_publisher(PoseStamped, "/aruco_target_pose", 10)
        self.block_center_pose_pub = self.create_publisher(PoseStamped, "/block_center_pose", 10)
        self.aruco_poses_sub = self.create_subscription(PoseArray, "/aruco_poses", self.aruco_poses_cb, 10)
        
        self.aruco_poses = None
        
        self.machine_state = "INIT"
        self.allow_execution = False
    
    def js_cb(self, msg):
        if len(msg.name) == 7:
            self.joint_pos.clear()
            for i in range(7):
                self.joint_pos.append(msg.position[i])
    
    def execute(self, value):
        self.allow_execution = value
    
    def aruco_poses_cb(self, msg):
        self.aruco_poses = msg
    
    def matrix_to_pose(self, M):
        pose = Pose()
        pose.position.x = M[0, 3]
        pose.position.y = M[1, 3]
        pose.position.z = M[2, 3]
        quat = tf_transformations.quaternion_from_matrix(M)
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]
        return pose
    
    # get first aruco pose
    def get_aruco_pose(self):
        if self.aruco_poses:
            if len(self.aruco_poses.poses) > 0:
                (sec, nanosec) = self.get_clock().now().seconds_nanoseconds()
                if sec - self.aruco_poses.header.stamp.sec > 1:
                    return None
                return self.aruco_poses.poses[0]
        return None

    # work callback
    def work_cb(self):
        if len(self.joint_pos) == 7:
            if self.machine_state == "INIT":
                self.get_logger().info('initializing ...')
                pan_tilt_deg_cmd = PanTiltCmdDeg()
                pan_tilt_deg_cmd.pitch = 15.0
                pan_tilt_deg_cmd.yaw = 0.0
                pan_tilt_deg_cmd.speed = 10
                self.pantil_pub.publish(pan_tilt_deg_cmd)
                
                if self.go_home_pos() and self.release():
                    self.get_logger().info('done.')
                    self.machine_state = "PREPARED"
                    self.valid_times = 0
                    time.sleep(1.0)
            elif self.machine_state == "PREPARED":
                current_aruco_pose = self.get_aruco_pose()
                if current_aruco_pose:
                    self.get_logger().info('aruco detected.')
                    
                    # Compute T_0c
                    T_0c = self.get_T_0c()
                    
                    # Compute T_ca
                    quat_ca = [current_aruco_pose.orientation.x, current_aruco_pose.orientation.y, current_aruco_pose.orientation.z, current_aruco_pose.orientation.w]
                    R_ca = tf_transformations.quaternion_matrix(quat_ca)
                    t_ca = np.array([current_aruco_pose.position.x, current_aruco_pose.position.y, current_aruco_pose.position.z])
                    T_ca = np.eye(4)
                    T_ca[:3, :3] = R_ca[:3, :3]
                    T_ca[:3, 3] = t_ca
                    
                    # Correct T_ca translation (from surface center to body center)
                    T_ca[0 : 3, 3] -= self.block_length / 2.0 * T_ca[0 : 3, 2]
                    
                    # T_0a = T_0c * T_ca
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
                    self.action_matrix = T_0a
                    
                    # Publish selected aruco pose
                    aruco_target_pose = PoseStamped()
                    aruco_target_pose.header.stamp = self.get_clock().now().to_msg()
                    aruco_target_pose.header.frame_id = "camera_color_optical_frame"
                    aruco_target_pose.pose = current_aruco_pose
                    self.aruco_target_pose_pub.publish(aruco_target_pose)
                    
                    # Publish block center pose
                    block_center_pose = PoseStamped()
                    block_center_pose.header.stamp = self.get_clock().now().to_msg()
                    block_center_pose.header.frame_id = "camera_color_optical_frame"
                    block_center_pose.pose = self.matrix_to_pose(T_ca)
                    self.block_center_pose_pub.publish(block_center_pose)
                    
                    # Publish arm target pose
                    arm_target_pose = PoseStamped()
                    arm_target_pose.header.stamp = self.get_clock().now().to_msg()
                    arm_target_pose.header.frame_id = "px100/base_link"
                    arm_target_pose.pose = self.matrix_to_pose(self.action_matrix)
                    self.arm_target_pose_pub.publish(arm_target_pose)
                    
                    # Test solution
                    mlist, solution_found, solution_valid, reached = self.matrix_control(self.action_matrix, custom_joints=[None, self.shoulder_offset, -1.4, 0.0])
                    self.get_logger().info(f'solution found: {solution_found}; solution valid: {solution_valid}.')
                    if solution_valid and self.allow_execution:
                        self.valid_times += 1
                        if self.valid_times > 3:
                            self.valid_times = 0
                            self.machine_state = "ACTION_WAIST"
            elif self.machine_state == "ACTION_WAIST":
                self.get_logger().info('action waist ...')
                mlist, solution_found, solution_valid, reached = self.matrix_control(self.action_matrix, custom_joints=[None, self.shoulder_offset, -1.4, 0.0])
                time.sleep(1.0)
                if reached:
                    self.get_logger().info('done.')
                    self.machine_state = "ACTION"
            elif self.machine_state == "ACTION":
                self.get_logger().info('action to target ...')
                mlist, solution_found, solution_valid, reached = self.matrix_control(self.action_matrix)
                time.sleep(1.0)
                if reached:
                    self.get_logger().info('done.')
                    self.gripper_controller(0.7, 1.0)
                    self.machine_state = "HOLD_UP"
            elif self.machine_state == "HOLD_UP":
                self.get_logger().info('holding up ...')
                if self.go_hand_up_pose():
                    self.get_logger().info('done.')
                    self.machine_state = "INIT"
            else:
                self.get_logger().info('unvalid machine state.')

    def get_T_0c(self):
        trans = self.tf_reader.get_arm_to_cam().transform
        translation = np.array([trans.translation.x, trans.translation.y, trans.translation.z])
        rotation = [trans.rotation.x, trans.rotation.y, trans.rotation.z, trans.rotation.w]
        R = tf_transformations.quaternion_matrix(rotation)
        matrix = np.eye(4)
        matrix[:3, :3] = R[:3, :3]
        matrix[:3, 3] = translation
        return np.linalg.inv(matrix)

    def set_single_pos(self, name, pos, blocking=True):
        '''
        ### @param: name: joint name
        ### @param: pos: radian
        ### @param: blocking - whether the arm need to check current position 

        '''
        self.arm_command.name = name
        self.arm_command.cmd = pos
        self.cmd_pub.publish(self.arm_command)

        thred = self.thred
        if blocking:
            check_pos = None
            cal_name = None
            if len(self.joint_pos) == 7:
                match name:
                    case "waist":
                        check_pos = self.joint_pos[0]
                        cal_name = 'joint'
                    case "shoulder":
                        check_pos = self.joint_pos[1]
                        cal_name = 'joint'
                    case "elbow":
                        check_pos = self.joint_pos[2]
                        cal_name = 'joint'
                    case "wrist_angle":
                        check_pos = self.joint_pos[3]
                        cal_name = 'joint'
                    case "gripper":
                        check_pos = self.joint_pos[4]
                        cal_name = 'gripper'
                    case _:
                        self.get_logger().info('unvalid name input!')

                match cal_name:
                    case "joint":
                        dis = np.abs(pos-check_pos)
                        if dis < thred:
                            return True
                        else:
                            self.get_logger().info('single joint moving...')
                            return False
                    case "gripper":
                        return True

        pass

    def set_group_pos(self, pos_list, blocking=True):
        '''
        ### @param: group pos: radian
        ### @param: blocking - whether the arm need to check current position 
        '''
        if len(pos_list) != self.num_joints:
            self.get_logger().info('unexpect length of list!')
        else:
            for i in range(self.num_joints):
                pos_list[i] = math.fmod(pos_list[i], 2 * math.pi)
                pos_list[i] = pos_list[i] - 2 * math.pi if pos_list[i] > math.pi else pos_list[i]
                pos_list[i] = pos_list[i] + 2 * math.pi if pos_list[i] < -math.pi else pos_list[i]
            
            self.arm_group_command.name = "arm"
            self.arm_group_command.cmd = pos_list
            self.group_pub.publish(self.arm_group_command)
     
            thred = self.thred
            if blocking:
                if len(self.joint_pos) == 7:
                    check_pos = self.joint_pos
                    if np.abs(pos_list[0] - check_pos[0]) < thred and np.abs(pos_list[1] - check_pos[1]) < thred and np.abs(pos_list[2] - check_pos[2]) < thred and np.abs(pos_list[3] - check_pos[3]) < thred:
                        return True
                    else:
                        return False
            pass

    def joint_to_pose(self, joint_state):
        return mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_state)

    def go_home_pos(self):
        state = self.set_group_pos([1.57, self.shoulder_offset, 1.5, -1.5])
        return state
    
    def go_hand_up_pose(self):
        state = self.set_group_pos([0.0, self.shoulder_offset, -1.5, 0.0])
        time.sleep(0.2)
        state = self.set_group_pos([0.0, self.shoulder_offset, -1.4, 0.0])
        return state

    def go_sleep_pos(self):
        state = self.set_group_pos([-1.4, -0.35, 0.7, 1.0])
        return state

    def matrix_control(self, T_sd, custom_guess: list[float]=None, execute: bool=True, custom_joints=None):
        if custom_guess is None:
            initial_guesses = self.initial_guesses
        else:
            initial_guesses = [custom_guess]

        for guess in initial_guesses:
            theta_list, success = mr.IKinSpace(
                Slist=self.robot_des.Slist,
                M=self.robot_des.M,
                T=T_sd,
                thetalist0=guess,
                eomg=0.1,
                ev=0.002,
            )
            # Check to make sure a solution was found and that no joint limits were violated
            solution_found = success
            solution_valid = False
            if success:
                theta_list = self._wrap_theta_list(theta_list)
                solution_valid = self._check_joint_limits(theta_list)
            
            reached = False
            if solution_valid:
                if execute:
                    joint_list = [theta_list[0], theta_list[1] + self.shoulder_offset, theta_list[2], theta_list[3]]
                    
                    # TODO BEGIN: 直接写死
                    if joint_list[0] > 0:
                        joint_list[0] *= 1.09
                    # TODO END
                    
                    if (custom_joints is not None) and (len(custom_joints) == self.num_joints):
                        for i in range(self.num_joints):
                            if custom_joints[i] is not None:
                                joint_list[i] = custom_joints[i]
                        reached = self.set_group_pos(joint_list)
                    
                    reached = self.set_group_pos(joint_list)
                    self.T_sb = T_sd
            return theta_list, solution_found, solution_valid, reached

        # self.core.get_logger().warn('No valid pose could be found. Will not execute')
        return theta_list, False, False

    def waist_control(self, pos):
        """
        lower limit = -1.5
        upper limit = 1.5
        """
        pos = float(pos)
        state = self.set_single_pos('waist', pos)
        return state
    
    def shoulder_control(self, pos):
        """
        lower limit = -0.4
        upper limit = ~0.9
        """
        pos = float(pos)
        state = self.set_single_pos('shoulder', pos)
        return state
    
    def elbow_control(self, pos):
        '''
        lower limit = -1.1
        upper limit = 0.8
        '''
        pos = float(pos)
        state = self.set_single_pos('elbow', pos)
        return state
    
    def wrist_control(self, pos):
        '''
        lower limit = -1.4
        upper limit = 1.8
        '''
        pos = float(pos)
        state = self.set_single_pos('wrist_angle', pos)
        return state

    def gripper_controller(self, effort, delay: float):
        '''
        effort: release = 1.5
        effort: grasp = -0.6
        '''
        name = 'gripper'
        effort = float(effort)
        if len(self.joint_pos) == 7:
            gripper_state = self.set_single_pos(name, effort)
            time.sleep(delay)
            return gripper_state

    def set_pressure(self, pressure: float) -> None:
        """
        Set the amount of pressure that the gripper should use when grasping an object.
        :param pressure: a scaling factor from 0 to 1 where the pressure increases as
            the factor increases
        """
        self.gripper_value = self.gripper_pressure_lower_limit + pressure * (
            self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit
        )

    def release(self, delay: float = 1.0) -> None:
        """
        Open the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(1.5, delay)
        return state

    def grasp(self, pressure: float = 0.5, delay: float = 1.0) -> None:
        """
        Close the gripper (when in 'pwm' control mode).
        :param delay: (optional) number of seconds to delay before returning control to the user
        """
        state = self.gripper_controller(pressure, delay)
        return state

    def _wrap_theta_list(self, theta_list: list[np.ndarray]) -> list[np.ndarray]:
        """
        Wrap an array of joint commands to [-pi, pi) and between the joint limits.

        :param theta_list: array of floats to wrap
        :return: array of floats wrapped between [-pi, pi)
        """
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
        if theta_list[0] > 1.5 or theta_list[0] < -1.5: # waist
            self.get_logger().info(f'waist out of range: {theta_list[0]}')
            return False
        if theta_list[1] > 0.9 or theta_list[1] < -0.4: # shoulder
            self.get_logger().info(f'shoulder out of range: {theta_list[1]}')
            return False
        if theta_list[2] > 1.7 or theta_list[2] < -1.6: # elbow
            self.get_logger().info(f'elbow out of range: {theta_list[2]}')
            return False
        if theta_list[3] > 1.8 or theta_list[3] < -1.8: # wrist
            self.get_logger().info(f'wrist out of range: {theta_list[3]}')
            return False
        return True


def main():
    try:
        rclpy.init(args = None)
        
        tf_reader = TF_Reader()
        while (tf_reader.get_arm_to_cam() == None):
            rclpy.spin_once(tf_reader)
        
        controller = ArmController(tf_reader)
        controller.execute(True)
        
        rclpy.spin(controller)
    finally:
        controller.get_logger().info('stopping ...')
        
        while not controller.go_home_pos():
            rclpy.spin_once(controller)
        
        tf_reader.destroy_node()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

