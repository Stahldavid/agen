import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class WebotsROS2Driver:
def __init__(self):
rclpy.init(args=None)
self.node = rclpy.create_node('webots_ros2_driver')

self.__robot = self.__robot.getDevice('driver')

self.__left_motor = self.__robot.getDevice('left wheel motor')
self.__right_motor = self.__robot.getDevice('right wheel motor')

self.__left_motor.setPosition(float('inf'))
self.__left_motor.setVelocity(0)

self.__right_motor.setPosition(float('inf'))
self.__right_motor.setVelocity(0)

self.__target_twist = Twist()

self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

self.timed_out = False


def __cmd_vel_callback(self, twist):
self.__target_twist = twist


def run(self):
while not self.timed_out:
rclpy.spin_once(self.node, timeout_sec=0)

forward_speed = self.__target_twist.linear.x
angular_speed = self.__target_twist.angular.z

command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

self.__left_motor.setVelocity(command_motor_left)
self.__right_motor.setVelocity(command_motor_right)


def set_timeout(self, timeout):
self.timed_out = timeout

if __name__ == '__main__':
driver = WebotsROS2Driver()
driver.run()
