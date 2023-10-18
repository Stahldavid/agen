import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist

MAX_RANGE = 0.15


class ObstacleAvoidance(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.create_subscription(Range, 'left_sensor', self.left_sensor_callback, 1)
        self.create_subscription(Range, 'right_sensor', self.right_sensor_callback, 1)

    def left_sensor_callback(self, message):
        self.left_sensor_value = message.range

    def right_sensor_callback(self, message):
        self.right_sensor_value = message.range
        command_message = Twist()
        command_message.linear.x = 0.1

        if self.left_sensor_value < 0.9 * MAX_RANGE or self.right_sensor_value < 0.9 * MAX_RANGE:
            command_message.angular.z = -2.0

        self.publisher.publish(command_message)


def main(args=None):
    rclpy.init(args=args)
    obstacle_avoidance = ObstacleAvoidance()
    rclpy.spin(obstacle_avoidance)
    obstacle_avoidance.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()