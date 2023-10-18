import rclpy
from rcl_interfaces.msg import Parameter
from rclpy.node import Node
from rclpy.parameter import get_parameter_value
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import os
import pathlib


class Control:
    def __init__(self):
        rclpy.init(args=None)
        self.__node = Node('control')

    def set_parameter(self, name, value):
        self.__node.set_parameters([Parameter(name=name, value=get_parameter_value(value))])

    def get_parameter(self, name):
        return self.__node.get_parameter(name).value

    def spawn_robot(self,package_dir='my_package',robot_description_path='resource/my_robot.urdf', world='worlds/my_world.wbt'):
        robot_description = pathlib.Path(os.path.join(package_dir, robot_description_path)).read_text()

        webots = WebotsLauncher(
            world=os.path.join(package_dir, world)
        )

        my_robot_driver = Node(
            package='webots_ros2_driver',
            executable='driver',
            output='screen',
            additional_env={'WEBOTS_CONTROLLER_URL': controller_url_prefix() + 'my_robot'},
            parameters=[
                {'robot_description': robot_description},
            ]
        )

        launch_description = LaunchDescription([
            webots,
            my_robot_driver,
        ])
        return launch_description

if __name__ == '__main__':
    control = Control()
    control.spawn_robot()