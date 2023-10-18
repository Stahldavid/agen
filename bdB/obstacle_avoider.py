import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist


MAX_RANGE = 0.15


class ObstacleAvoider(Node):
    def __init__(self):
        super().__init__('obstacle_avoider_node')
        self.__publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Range, 'left_sensor', self.__left_sensor_callback, 10)
        self.create_subscription(Range, 'right_sensor', self.__right_sensor_callback, 10)

    def __left_sensor_callback(self, message):
        pass

    def __right_sensor_callback(self, message):
        pass

    def avoid_obstacles(self):
        rate = self.create_rate(10)
        while rclpy.ok():
            pass


def main():
    rclpy.init()
    avoider = ObstacleAvoider()
    avoider.avoid_obstacles()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
