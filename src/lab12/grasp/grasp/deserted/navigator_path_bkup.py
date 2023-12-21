import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

from ..Astar_planner import AStarPlanner


class MapReceiver(Node):
    def __init__(self):
        super().__init__('navigator_map_receiver')
        
        self.path_pub = self.create_publisher(Path, 'myplan', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, QoSProfile(depth = 1, reliability = ReliabilityPolicy.BEST_EFFORT))
        self.map = None
    
    def map_callback(self, msg):
        self.map = msg


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
    
    # map
    map_receiver = MapReceiver()
    while map_receiver.map is None:
        map_receiver.get_logger().info('waiting for /map ...')
        # navigator.changeMap('/home/tony/MyFiles_ClickHere/Workspace/gralerfics/data/map.yaml')
        rclpy.spin_once(map_receiver, timeout_sec = 0.1)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active()
    
    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    # Go to our demos first goal pose
    goal_pose = PoseStamped()
    # goal_pose.header.frame_id = 'map'
    # goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    # goal_pose.pose.position.x = 0.3066274822848437
    # goal_pose.pose.position.y = -3.2658465986026695
    # goal_pose.pose.orientation.z = -0.6686671777907486
    # goal_pose.pose.orientation.w = 0.7435618369344646
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.7667487875336247
    goal_pose.pose.position.y = -3.220366191076802
    goal_pose.pose.orientation.z = -0.007822402212393793
    goal_pose.pose.orientation.w = 0.9999694045437728

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)

    planner = AStarPlanner(map_receiver.map, 0.2)
    
    navigator.get_logger().info('planning ...')
    
    initial_x, initial_y = initial_pose.pose.position.x, initial_pose.pose.position.y
    goal_x, goal_y = goal_pose.pose.position.x, goal_pose.pose.position.y
    rx, ry = planner.planning(initial_x, initial_y, goal_x, goal_y)
    path: Path = Path()
    path.header.stamp = navigator.get_clock().now().to_msg()
    path.header.frame_id = 'map'
    n = len(rx)
    for i in range(20, n):
        pose = PoseStamped()
        pose.pose.position.x, pose.pose.position.y = rx[i], ry[i]
        if i == n - 1:
            pose.pose.orientation = goal_pose.pose.orientation
        path.poses.append(pose)
    map_receiver.path_pub.publish(path)
    
    # navigator.goToPose(goal_pose)
    navigator.followPath(path)

    i = 0
    while not navigator.isTaskComplete():
        ##################################################
        #                                                #
        # Implement some code here for your application! #
        #                                                #
        ##################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        # if feedback and i % 5 == 0:
        #     print(
        #         'Estimated time of arrival: '
        #         + '{0:.0f}'.format(
        #             Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
        #             / 1e9
        #         )
        #         + ' seconds.'
        #     )

        #     Some navigation timeout to demo cancellation
        #     if Duration.from_msg(feedback.navigation_time) > Duration(seconds=600.0):
        #         navigator.cancelTask()

        #     Some navigation request change to demo preemption
        #     if Duration.from_msg(feedback.navigation_time) > Duration(seconds=18.0):
        #         goal_pose.pose.position.x = -3.0
        #         navigator.goToPose(goal_pose)

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    # navigator.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()