#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.parameter import get_parameter_value
from rcl_interfaces.msg import Parameter
from rclpy.duration import Duration

from controller_manager import (
    configure_controller,
    list_controllers,
    load_controller,
    switch_controllers,
    unload_controller,
)

from ros2param.api import call_set_parameters
from ros2param.api import load_parameter_file


class ROS2Controller(Node):
    def __init__(self):
        super().__init__('ros2_controller')

        # initial configuration
        self.init_controller()

    def init_controller(self):
        # TODO: configure your controllers here
        pass

    def run_controller(self, controller):
        # TODO: implement run_controller logic here
        pass


def main(args=None):
    rclpy.init(args=args)

    # create an instance of ROS2Controller
    controller = ROS2Controller()

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass

    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
