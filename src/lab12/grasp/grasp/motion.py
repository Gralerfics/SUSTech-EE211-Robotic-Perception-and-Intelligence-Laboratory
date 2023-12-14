import time

import rclpy

from .base_controller import BaseController


def main():
    try:
        rclpy.init(args = None)

        base_controller = BaseController()
        rclpy.spin(base_controller)
    finally:
        base_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

