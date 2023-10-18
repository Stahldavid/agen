import rclpy
from geometry_msgs.msg import Twist

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025

class MyRobotDriver:
    def __init__(self, webots_node, properties):
        self.robot = webots_node.robot

        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')

        self.left_motor.setPosition(float('inf'))
        self.left_motor.setVelocity(0)

        self.right_motor.setPosition(float('inf'))
        self.right_motor.setVelocity(0)

        self.target_twist = Twist()

        rclpy.init(args=None)
        self.node = rclpy.create_node('my_robot_driver')
        self.node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 1)

    def __cmd_vel_callback(self, twist):
        self.target_twist = twist

    def step(self):
        rclpy.spin_once(self.node, timeout_sec=0)

        forward_speed = self.target_twist.linear.x
        angular_speed = self.target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        self.left_motor.setVelocity(command_motor_left)
        self.right_motor.setVelocity(command_motor_right)