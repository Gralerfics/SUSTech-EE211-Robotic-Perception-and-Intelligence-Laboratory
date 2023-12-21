import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from geometry_msgs.msg import PoseStamped, Twist

from grasp_interfaces.srv import GraspAction, GraspQuery


def go_goal(nav, goal, blocking = True):
    nav.goToPose(goal)
    if blocking:
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
        return nav.getResult()
    else:
        return None


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.3025282477783394
    initial_pose.pose.position.y = -0.36741189949949404
    initial_pose.pose.orientation.z = -0.7140501663813629
    initial_pose.pose.orientation.w = 0.7000945363954414
    navigator.setInitialPose(initial_pose)
    
    # navigator.lifecycleStartup()
    navigator.waitUntilNav2Active()

    # You may use the navigator to clear or obtain costmaps
    navigator.clearAllCostmaps() # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # services
    grasp_action_client = navigator.create_client(GraspAction, 'grasp_action')
    grasp_query_solved_client = navigator.create_client(GraspQuery, 'grasp_is_solved')
    grasp_query_holding_client = navigator.create_client(GraspQuery, 'grasp_is_holding')
    grasp_action_client.wait_for_service()
    grasp_query_solved_client.wait_for_service()
    grasp_query_holding_client.wait_for_service()
    
    # publisher
    cmd_vel_pub = navigator.create_publisher(Twist, '/cmd_vel', 10)
    
    # block pose subscriber
    block_center_pose = None
    block_center_pose_expire_time = 2.0
    
    def block_center_pose_callback(msg):
        nonlocal block_center_pose
        block_center_pose = msg
    block_center_pose_sub = navigator.create_subscription(PoseStamped, '/block_center_pose', block_center_pose_callback, 10)
    
    def get_block_insight_pose():
        nonlocal block_center_pose
        if block_center_pose.header.stamp.sec + block_center_pose_expire_time < navigator.get_clock().now().to_msg().sec:
            return None
        return block_center_pose
    
    # Reset
    # request = GraspAction.Request()
    # request.action = 'reset'
    # future = grasp_action_client.call_async(request)
    # rclpy.spin_until_future_complete(navigator, future)
    # response = future.result()
    # time.sleep(1.0)
    
    # Go to A
    a_goal = PoseStamped()
    a_goal.header.frame_id = 'map'
    a_goal.header.stamp = navigator.get_clock().now().to_msg()
    a_goal.pose.position.x = 0.2266274822848437
    a_goal.pose.position.y = -3.4258465986026695
    a_goal.pose.orientation.z = -0.6686671777907486
    a_goal.pose.orientation.w = 0.7435618369344646
    go_goal(navigator, a_goal, blocking = False)
    
    # Check aruco insight
    insight = False
    while not navigator.isTaskComplete():
        block_pose = get_block_insight_pose()
        if block_pose is not None:
            insight = True
            break
    
    # Approach to block
    if insight:
        block_pose = get_block_insight_pose()
        
        navigator.cancelTask()
        # while not navigator.isTaskComplete(): # TODO: temporarily substitute cancelTask()
        #     feedback = navigator.getFeedback()
        
        
    else:
        pass # TODO: look around
    
    # Grasp
    request = GraspAction.Request()
    request.action = 'grasp'
    future = grasp_action_client.call_async(request)
    rclpy.spin_until_future_complete(navigator, future)
    response = future.result()
    
    while True:
        request = GraspQuery.Request()
        future = grasp_query_holding_client.call_async(request)
        rclpy.spin_until_future_complete(navigator, future)
        response = future.result()
        
        if response.result:
            navigator.get_logger().info(f'held.')
            break
        time.sleep(0.5)

    # Face to B
    a_goal.pose.orientation.z = -0.007822402212393793
    a_goal.pose.orientation.w = 0.9999694045437728
    go_goal(navigator, a_goal)
    
    # Go to B
    b_goal = PoseStamped()
    b_goal.header.frame_id = 'map'
    b_goal.header.stamp = navigator.get_clock().now().to_msg()
    b_goal.pose.position.x = 2.8567487875336247
    b_goal.pose.position.y = -3.320366191076802
    b_goal.pose.orientation.z = -0.007822402212393793
    b_goal.pose.orientation.w = 0.9999694045437728
    go_goal(navigator, b_goal)
    
    # Release
    request = GraspAction.Request()
    request.action = 'release'
    future = grasp_action_client.call_async(request)
    rclpy.spin_until_future_complete(navigator, future)
    response = future.result()
    
    # Face to S
    b_goal.pose.orientation.z = 0.924  # 0.9985
    b_goal.pose.orientation.w = 0.383  # 0.0547
    go_goal(navigator, b_goal)
    
    # Go to S
    s_goal = PoseStamped()
    s_goal.header.frame_id = 'map'
    s_goal.header.stamp = navigator.get_clock().now().to_msg()
    s_goal.pose.position.x = 0.3725282477783394
    s_goal.pose.position.y = -0.45741189949949404
    s_goal.pose.orientation.z = -0.7000945363954414
    s_goal.pose.orientation.w = -0.7140501663813629
    go_goal(navigator, s_goal)

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()

