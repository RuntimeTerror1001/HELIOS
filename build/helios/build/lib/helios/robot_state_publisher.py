import rclpy
from rclpy.node import Node

class Robot_State_Publisher(Node):
    def __init__(self):
        super().__init__('robot_state_publisher')

def main(args=None):
    rclpy.init(args=args)
    node = Robot_State_Publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
