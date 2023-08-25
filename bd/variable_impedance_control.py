import rclpy
from geometry_msgs.msg import Twist
from controller_manager import (
    configure_controller,
    list_controllers,
    load_controller,
    switch_controllers,
    unload_controller
)

from my_robot_driver import MyRobotDriver
from obstacle_avoider import ObstacleAvoider


class VariableImpedanceControl:
    def __init__(self):
        rclpy.init()

        self.robot_driver = MyRobotDriver()

        self.obstacle_avoider = ObstacleAvoider()

    def run(self):
        while rclpy.ok():
            self.robot_driver.step()

        rclpy.shutdown()


def main(args=None):
    variable_impedance_control = VariableImpedanceControl()
    variable_impedance_control.run()


if __name__ == '__main__':
    main()