import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from geometry_msgs.msg import PoseStamped, Twist

from grasp_interfaces.srv import GraspAction, GraspQuery


class TemporaryCommander(Node):
    def __init__(self):
        super().__init__('temporary_commander')
        
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
    
    pass


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
    
    def get_block_center_pose(self):
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

    # Nodes
    navigator = MyNavigator()
    temporary_client = TemporaryClient()
    temporary_commander = TemporaryCommander()

    # Initialization
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.3025282477783394
    initial_pose.pose.position.y = -0.36741189949949404
    initial_pose.pose.orientation.z = -0.7140501663813629
    initial_pose.pose.orientation.w = 0.7000945363954414
    navigator.initialize(initial_pose)
    
    # Go to A
    a_goal = PoseStamped()
    a_goal.header.frame_id = 'map'
    a_goal.header.stamp = navigator.get_clock().now().to_msg()
    a_goal.pose.position.x = 0.2266274822848437
    a_goal.pose.position.y = -3.4258465986026695
    a_goal.pose.orientation.z = -0.6686671777907486
    a_goal.pose.orientation.w = 0.7435618369344646
    navigator.go_to_goal(a_goal, blocking = False)
    
    # Check aruco insight
    insight = False
    while not navigator.isTaskComplete():
        temporary_client.get_logger().info(f'checking aruco insight ...')
        block_pose = temporary_client.get_block_center_pose()
        if block_pose is not None:
            insight = True
            temporary_client.get_logger().info(f'aruco detected.')
            break
    
    # Approach to block
    if insight:
        block_pose = temporary_client.get_block_center_pose()
        
        # navigator.cancelTask()
        while not navigator.isTaskComplete(): # TODO: temporarily substitute cancelTask()
            feedback = navigator.getFeedback()
        
        pass
    else:
        pass # TODO: look around
    
    # Grasp
    temporary_client.grasp_action('grasp')
    
    while not temporary_client.grasp_query_holding():
        time.sleep(0.5)
    temporary_client.get_logger().info(f'held.')
        
    # Judge whether the grasp is successful (by checking the aruco insight)

    # Face to B
    a_goal.pose.orientation.z = -0.007822402212393793
    a_goal.pose.orientation.w = 0.9999694045437728
    navigator.go_to_goal(a_goal)
    
    # Go to B
    b_goal = PoseStamped()
    b_goal.header.frame_id = 'map'
    b_goal.header.stamp = navigator.get_clock().now().to_msg()
    b_goal.pose.position.x = 2.8567487875336247
    b_goal.pose.position.y = -3.320366191076802
    b_goal.pose.orientation.z = -0.007822402212393793
    b_goal.pose.orientation.w = 0.9999694045437728
    navigator.go_to_goal(b_goal)
    
    # Release
    temporary_client.grasp_action('release')
    
    # Face to S
    b_goal.pose.orientation.z = 0.924  # 0.9985
    b_goal.pose.orientation.w = 0.383  # 0.0547
    navigator.go_to_goal(b_goal)
    
    # Go to S
    s_goal = PoseStamped()
    s_goal.header.frame_id = 'map'
    s_goal.header.stamp = navigator.get_clock().now().to_msg()
    s_goal.pose.position.x = 0.3725282477783394
    s_goal.pose.position.y = -0.45741189949949404
    s_goal.pose.orientation.z = -0.7000945363954414
    s_goal.pose.orientation.w = -0.7140501663813629
    navigator.go_to_goal(s_goal)

    navigator.shutdown()


if __name__ == '__main__':
    main()

