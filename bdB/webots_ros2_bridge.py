import rclpy
from rclpy.node import Node


class WebotsROS2Bridge(Node):
    def __init__(self):
        super().__init__('webots_ros2_bridge')

        # Add your code here

    def run(self):
        # Add your code here
        pass

def main(args=None):
    rclpy.init(args=args)
    bridge_node = WebotsROS2Bridge()
    bridge_node.run()
    bridge_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
