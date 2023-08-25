import rclpy
from rclpy.node import Node


class Ros2ControlInterface(Node):
    def __init__(self):
        super().__init__('ros2_control_interface')

        # Add your code here

    def run(self):
        # Add your code here
        pass


def main(args=None):
    rclpy.init(args=args)
    control_node = Ros2ControlInterface()
    control_node.run()
    control_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()