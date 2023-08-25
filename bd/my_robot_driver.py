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

# from https://stackoverflow.com/a/287944


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


def first_match(iterable, predicate):
    return next((n for n in iterable if predicate(n)), None)


def wait_for_value_or(function, node, timeout, default, description):
    while node.get_clock().now() < timeout:
        if result := function():
            return result
        node.get_logger().info(
            f"Waiting for {description}", throttle_duration_sec=2, skip_first=True
        )
        time.sleep(0.2)
    return default


def combine_name_and_namespace(name_and_namespace):
    node_name, namespace = name_and_namespace
    return namespace + ("" if namespace.endswith("/") else "/") + node_name


def find_node_and_namespace(node, full_node_name):
    node_names_and_namespaces = node.get_node_names_and_namespaces()
    return first_match(
        node_names_and_namespaces, lambda n: combine_name_and_namespace(n) == full_node_name
    )


def has_service_names(node, node_name, node_namespace, service_names):
    client_names_and_types = node.get_service_names_and_types_by_node(node_name, node_namespace)
    if not client_names_and_types:
        return False
    client_names, _ = zip(*client_names_and_types)
    return all(service in client_names for service in service_names)


def wait_for_controller_manager(node, controller_manager, timeout_duration):
    # List of service names from controller_manager we wait for
    service_names = (
        f"{controller_manager}/configure_controller",
        f"{controller_manager}/list_controllers",
        f"{controller_manager}/list_controller_types",
        f"{controller_manager}/list_hardware_components",
        f"{controller_manager}/list_hardware_interfaces",
        f"{controller_manager}/load_controller",
        f"{controller_manager}/reload_controller_libraries",
        f"{controller_manager}/switch_controller",
        f"{controller_manager}/unload_controller",
    )

    # Wait for controller_manager
    timeout = node.get_clock().now() + Duration(seconds=timeout_duration)
    node_and_namespace = wait_for_value_or(
        lambda: find_node_and_namespace(node, controller_manager),
        node,
        timeout,
        None,
        f"'"{controller_manager}"' node to exist",
    )

class MyRobotDriver:
    HALF_DISTANCE_BETWEEN_WHEELS = 0.045
    WHEEL_RADIUS = 0.025

    def __init__(self, controller):
        self.robot = self.controller.get_robot()
        self.left_wheel = self.robot.getDevice('left wheel motor')
        self.right_wheel = self.robot.getDevice('right wheel motor')
        self.left_wheel.setPosition(float('inf'))
        self.left_wheel.setVelocity(0)
        self.right_wheel.setPosition(float('inf'))
        self.right_wheel.setVelocity(0)

        self.target_twist = Twist()

        rclpy.init(args=None)
        self.node = rclpy.create_node('my_robot_driver')
        self.node.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 1)

    def cmd_vel_callback(self, twist):
        self.target_twist = twist

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        forward_speed = self.target_twist.linear.x
        angular_speed = self.target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * self.HALF_DISTANCE_BETWEEN_WHEELS) / self.WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * self.HALF_DISTANCE_BETWEEN_WHEELS) / self.WHEEL_RADIUS

        self.left_wheel.setVelocity(command_motor_left)
        self.right_wheel.setVelocity(command_motor_right)