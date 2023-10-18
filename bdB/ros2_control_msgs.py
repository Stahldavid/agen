from geometry_msgs.msg import Twist
import rclpy
from rcl_interfaces.msg import Parameter
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.parameter import get_parameter_value
from rclpy.signals import SignalHandlerOptions
from ros2param.api import call_set_parameters
from ros2param.api import load_parameter_file

class ros2_control_msgs(Node):
    def __init__(self):
        super().__init__('ros2_control_msgs')

        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.get_logger().info('Publisher created for cmd_vel topic')

    def publish_message(self, linear_x, angular_z):
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = angular_z
        self.__publisher.publish(twist)
        self.get_logger().info('Publishing Twist message')

    def read_parameter(self, param_name):
        param = self.get_parameter(param_name)
        if param.type_ == Parameter.Type.NOT_SET:
            raise Exception(f'Parameter {param_name} is not set')
        return param.value
