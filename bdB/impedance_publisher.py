import rclpy
from std_msgs.msg import Float64
from webots_ros2_core.webots_node import WebotsNode


class ImpedancePublisher(WebotsNode):
    def __init__(self, args):
        super().__init__('impedance_publisher', args)
        # Create and initialize necessary variables
        self.__impedance_publisher = self.create_publisher(
            Float64,
            'impedance',
            1,
        )

    def loop(self):
        # TODO: Implement the main loop
        pass


def main(args=None):
    rclpy.init(args=args)
    publisher = ImpedancePublisher(args=args)
    publisher.loop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()