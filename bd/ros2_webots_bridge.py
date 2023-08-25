import rclpy
from webots_ros2.webots.simulation import WebotsNode

class ROS2WebotsBridge:
    def __init__(self):
        self.node = WebotsNode('ros2_webots_bridge')

    def run(self):
        while rclpy.ok():
            # Bridge logic between ROS2 and Webots
            pass


def main(args=None):
    rclpy.init(args=args)
    bridge = ROS2WebotsBridge()
    bridge.run()
    bridge.node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()