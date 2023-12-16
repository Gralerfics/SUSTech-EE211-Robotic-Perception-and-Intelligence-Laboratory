import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist, Pose

from .base_controller import BaseController


def main():
    try:
        rclpy.init(args = None)

        base_controller = BaseController()
        
        # publish initial pose
        while base_controller.current_pose is None:
            time.sleep(1.0)
            base_controller.get_logger().info('publishing initial pose and waiting for amcl_pose ...')
            initial_pose = PoseWithCovarianceStamped()
            initial_pose.header.frame_id = 'map'
            initial_pose.header.stamp = base_controller.get_clock().now().to_msg()
            initial_pose.pose.pose.position.x = 0.3025282477783394
            initial_pose.pose.pose.position.y = -0.36741189949949404
            initial_pose.pose.pose.orientation.z = -0.7140501663813629
            initial_pose.pose.pose.orientation.w = 0.7000945363954414
            base_controller.initialpose_pub.publish(initial_pose)
            rclpy.spin_once(base_controller)
        
        # S -> A
        S_A_goal = Pose()
        S_A_goal.position.x = 0.3066274822848437
        S_A_goal.position.y = -3.2658465986026695
        S_A_goal.orientation.z = -0.6686671777907486
        S_A_goal.orientation.w = 0.7435618369344646
        base_controller.set_goal(S_A_goal)
        while not base_controller.is_goal_reached():
            base_controller.get_logger().info('hahaha')
            base_controller.velocity_controller()
            rclpy.spin_once(base_controller, timeout_sec = 0.1)
    finally:
        base_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

