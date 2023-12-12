from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import rclpy
from rclpy.node import Node

class TF_Reader(Node):
    def __init__(self):
        super().__init__("tf_reader")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.read_timer = self.create_timer(0.01,self.timer_cb)
        pass
    def timer_cb(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform("camera_link","px100/base_link",now)
            print(trans)
        except :
            pass

        pass

def main():
    rclpy.init(args=None)
    node = TF_Reader()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()

