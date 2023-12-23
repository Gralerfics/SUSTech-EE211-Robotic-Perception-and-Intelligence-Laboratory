import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from geometry_msgs.msg import PoseStamped, Twist

from grasp_interfaces.srv import GraspAction, GraspQuery


def go_goal_blocking(nav, goal):
    nav.goToPose(goal)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
    return nav.getResult()


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
    # initial_pose.pose.position.x = 0.2266274822848437
    # initial_pose.pose.position.y = -3.4258465986026695
    # initial_pose.pose.orientation.z = -0.6686671777907486
    # initial_pose.pose.orientation.w = 0.7435618369344646
    navigator.setInitialPose(initial_pose)
    
    # navigator.lifecycleStartup()
    navigator.waitUntilNav2Active()

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    grasp_action_client = navigator.create_client(GraspAction, 'grasp_action')
    grasp_query_solved_client = navigator.create_client(GraspQuery, 'grasp_is_solved')
    grasp_query_holding_client = navigator.create_client(GraspQuery, 'grasp_is_holding')
    grasp_action_client.wait_for_service()
    grasp_query_solved_client.wait_for_service()
    grasp_query_holding_client.wait_for_service()
    
    cmd_vel_pub = navigator.create_publisher(Twist, 'cmd_vel', 10)

    # A
    a_goal = PoseStamped()
    a_goal.header.frame_id = 'map'
    a_goal.header.stamp = navigator.get_clock().now().to_msg()
    a_goal.pose.position.x = 0.1866274822848437 # 0.2266274822848437
    a_goal.pose.position.y = -3.3058465986026695 # -3.4258465986026695
    a_goal.pose.orientation.z = -0.6686671777907486
    a_goal.pose.orientation.w = 0.7435618369344646
    go_goal_blocking(navigator, a_goal)
    
    # request = GraspAction.Request()
    # request.action = 'reset'
    # future = grasp_action_client.call_async(request)
    # rclpy.spin_until_future_complete(navigator, future)
    # response = future.result()
    # time.sleep(1.0)
    
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
        
        navigator.get_logger().info(f'is_holding: {response.result}')
        if response.result:
            break
        time.sleep(0.5)

    a_goal.pose.orientation.z = -0.007822402212393793
    a_goal.pose.orientation.w = 0.9999694045437728
    go_goal_blocking(navigator, a_goal)
    # vel_cmd = Twist()
    # vel_cmd.linear.x = 0.0
    # vel_cmd.angular.z = 3.1415926 / 4
    # cmd_vel_pub.publish(vel_cmd)
    # time.sleep(2.0)
    # cmd_vel_pub.publish(Twist())
    
    # B
    b_goal = PoseStamped()
    b_goal.header.frame_id = 'map'
    b_goal.header.stamp = navigator.get_clock().now().to_msg()
    b_goal.pose.position.x = 2.8567487875336247
    b_goal.pose.position.y = -3.320366191076802
    b_goal.pose.orientation.z = -0.007822402212393793
    b_goal.pose.orientation.w = 0.9999694045437728
    go_goal_blocking(navigator, b_goal)
    
    request = GraspAction.Request()
    request.action = 'release'
    future = grasp_action_client.call_async(request)
    rclpy.spin_until_future_complete(navigator, future)
    response = future.result()
    
    b_goal.pose.orientation.z = 0.924 # 0.9985
    b_goal.pose.orientation.w = 0.383 # 0.0547
    go_goal_blocking(navigator, b_goal)
    # vel_cmd = Twist()
    # vel_cmd.linear.x = 0.0
    # vel_cmd.angular.z = 3.1415926 / 4
    # cmd_vel_pub.publish(vel_cmd)
    # time.sleep(3.0)
    # cmd_vel_pub.publish(Twist())
    
    # S
    s_goal = PoseStamped()
    s_goal.header.frame_id = 'map'
    s_goal.header.stamp = navigator.get_clock().now().to_msg()
    s_goal.pose.position.x = 0.3025282477783394 # 0.3725282477783394
    s_goal.pose.position.y = -0.40741189949949404 # -0.45741189949949404
    s_goal.pose.orientation.z = 0.924 # -0.7000945363954414
    s_goal.pose.orientation.w = 0.383 # -0.7140501663813629
    go_goal_blocking(navigator, s_goal)

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()

