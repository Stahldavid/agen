import rclpy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

HALF_DISTANCE_BETWEEN_WHEELS = 0.045
WHEEL_RADIUS = 0.025


class MyRobotDriver:
    def __init__(self):
        self.__target_twist = Twist()
        self.__node = rclpy.create_node('my_robot_driver_node')
        self.__subscriber = self.__node.create_subscription(Twist, 'cmd_vel', self.__cmd_vel_callback, 10)
        self.__left_motor = self.__node.create_publisher(Float64, 'left_wheel_motor/command', 10)
        self.__right_motor = self.__node.create_publisher(Float64, 'right_wheel_motor/command', 10)

    def __cmd_vel_callback(self, twist):
        self.__target_twist = twist

    def calculate_wheel_commands(self):
        forward_speed = self.__target_twist.linear.x
        angular_speed = self.__target_twist.angular.z

        command_motor_left = (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS
        command_motor_right = (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) / WHEEL_RADIUS

        return command_motor_left, command_motor_right

    def publish_wheel_commands(self):
        rate = self.__node.create_rate(10)
        while rclpy.ok():
            command_motor_left, command_motor_right = self.calculate_wheel_commands()

            left_wheel_command = Float64()
            left_wheel_command.data = command_motor_left
            right_wheel_command = Float64()
            right_wheel_command.data = command_motor_right

            self.__left_motor.publish(left_wheel_command)
            self.__right_motor.publish(right_wheel_command)

            rate.sleep()

def main():
    rclpy.init()
    driver = MyRobotDriver()
    driver.publish_wheel_commands()
    rclpy.shutdown()


if __name__ == '__main__':
    main()