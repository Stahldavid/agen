#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String
from webots_ros2_core.webots_robot import WebotsRobot


class MyRobot(WebotsRobot):
    def __init__(self, args=None):
        super().__init__(args)
        self.sensor = None

    def init_sensors(self):
        self.sensor = self.robot.getDevice('my_sensor')

    def step(self):
        if not self.sensor:
            return
        msg = String()
        msg.data = str(self.sensor.getValue())
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    try:
        my_robot = MyRobot('my_robot', options=None) 
        my_robot.spin()
    finally:
        my_robot.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
