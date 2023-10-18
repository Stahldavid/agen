#!/usr/bin/env python3
import rclpy


class Ros2WebotsInterface:
    def __init__(self):
        rclpy.init(args=None)
        self.node = rclpy.create_node('ros2_webots_interface')

    def run(self):
        rclpy.spin(self.node)

def main():
    interface = Ros2WebotsInterface()
    interface.run()

if __name__ == '__main__':
    main()