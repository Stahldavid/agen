import rclpy
from std_msgs.msg import Float64
from webots_ros2_core.webots_node import WebotsNode
from webots_ros2_msgs.msg import Float64Stamped


class ForceFeedbackPublisher(WebotsNode):
    def __init__(self, args):
        super().__init__('force_feedback_publisher', args)
        # Create and initialize necessary variables
        self.__force_feedback_publisher = self.create_publisher(
            Float64Stamped,
            'force_feedback',
            1,
        )

    def loop(self):
        # TODO: Implement the main loop
        pass


def main(args=None):
    rclpy.init(args=args)
    publisher = ForceFeedbackPublisher(args=args)
    publisher.loop()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
