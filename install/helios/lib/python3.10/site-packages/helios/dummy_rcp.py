import rclpy
from rclpy.node import Node

class DummyRSP(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

def main(args=None):
    rclpy.init(args=args)
    node = DummyRSP()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
