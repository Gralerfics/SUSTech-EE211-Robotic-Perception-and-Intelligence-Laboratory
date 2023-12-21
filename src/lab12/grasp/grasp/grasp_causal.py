import time

import rclpy

from .tf_reader import TF_Reader
from .arm_controller_causal import ArmController


def main():
    try:
        rclpy.init(args = None)
        
        # wait for tf to be ready
        tf_reader = TF_Reader()
        while (tf_reader.get_arm_to_cam() == None):
            rclpy.spin_once(tf_reader)
        
        arm_controller = ArmController(tf_reader)
        rclpy.spin(arm_controller)
    finally:
        tf_reader.destroy_node()
        arm_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

