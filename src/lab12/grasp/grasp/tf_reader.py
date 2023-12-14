import rclpy
from rclpy.node import Node

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class TF_Reader(Node):
    def __init__(self):
        super().__init__("tf_reader")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.read_timer = self.create_timer(0.5, self.timer_cb)
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

