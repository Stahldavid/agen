import argparse
import errno
import os
import sys
import time
import warnings
import io
from contextlib import redirect_stdout, redirect_stderr

from controller_manager import (
      configure_controller,
      list_controllers,
      load_controller,
      switch_controllers,
      unload_controller,
)

import rclpy
from rcl_interfaces.msg import Parameter
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import get_parameter_value
from rclpy.signals import SignalHandlerOptions
from ros2param.api import call_set_parameters
from ros2param.api import load_parameter_file


class ControllerManager(Node):
    def __init__(self):
        super().__init__('controller_manager')

        self.create_subscription(Parameter, "/controller_manager/parameter_events", self.__parameter_event_callback, 1)

    def configure_controller(self, name, config):
        configure_controller(name, config)

    def list_controllers(self):
        return list_controllers()

    def load_controller(self, name, controller_type):
        load_controller(name, controller_type)

    def switch_controllers(
        self,
        start_controllers: list,
        stop_controllers: list,
        strictness=SwitchControllerRequest.STRICT,
        timeout=None,
    ):
        switch_controllers(start_controllers, stop_controllers, strictness, timeout)

    def unload_controller(self, name):
        unload_controller(name)

    def __parameter_event_callback(self, parameter_event):
        # Process parameter event
        pass