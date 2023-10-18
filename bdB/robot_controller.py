import argparse
import rclpy
from rclpy.node import Node


class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')

    def run(self):
        pass


def main(args=None):
    parser = argparse.ArgumentParser()
    args = parser.parse_args(args)

    rclpy.init(args=args)
    robot_controller = RobotController()
    robot_controller.run()

    rclpy.spin(robot_controller)
    rclpy.shutdown()


if __name__ == '__main__':
    main()