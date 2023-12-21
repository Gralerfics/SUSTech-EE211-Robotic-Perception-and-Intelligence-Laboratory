import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

from .Astar_planner import AStarPlanner
from .my_controller import LocalController


class MapReceiver(Node):
    def __init__(self):
        super().__init__('navigator_map_receiver')
        
        # self.path_pub = self.create_publisher(Path, 'myplan', 10)
        self.map_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, QoSProfile(depth = 1, reliability = ReliabilityPolicy.BEST_EFFORT))
        self.map = None
    
    def map_callback(self, msg):
        self.map = msg


def main():
    rclpy.init()
    
    # map
    map_receiver = MapReceiver()
    while map_receiver.map is None:
        map_receiver.get_logger().info('waiting for /map ...')
        # navigator.changeMap('/home/tony/MyFiles_ClickHere/Workspace/gralerfics/data/map.yaml')
        rclpy.spin_once(map_receiver)
    
    navigator = BasicNavigator()

    # Set the initial pose S
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.3025282477783394
    initial_pose.pose.position.y = -0.36741189949949404
    initial_pose.pose.orientation.z = -0.7140501663813629
    initial_pose.pose.orientation.w = 0.7000945363954414
    navigator.setInitialPose(initial_pose)


    # Go to our demos first goal pose A
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = navigator.get_clock().now().to_msg()
    goal_pose.pose.position.x = 2.7667487875336247
    goal_pose.pose.position.y = -3.220366191076802
    goal_pose.pose.orientation.z = -0.007822402212393793
    goal_pose.pose.orientation.w = 0.9999694045437728

    # sanity check a valid path exists
    # path = navigator.getPath(initial_pose, goal_pose)
    
    planner = AStarPlanner(map_receiver.map, 0.2)
    controller = LocalController()
    path = Path()
    path = planner.find_path(initial_pose, goal_pose)
    # map_receiver.path_pub.publish(path)
    
    controller.goToGoal(path)

    # exit(0)


if __name__ == '__main__':
    main()
    