import rclpy
from std_msgs.msg import Float64


class Ros2Control:
    """A class to control the impedance controller using ROS2."

    def __init__(self):
        """Initialize the ROS2 node, publisher, and subscription."
        rclpy.init(args=None)
        self.__node = rclpy.create_node('ros2_control')
        self.publisher = self.__node.create_publisher(Float64, 'impedance_params', 1)
        self.subscription = self.__node.create_subscription(Float64, 'force_feedback', self.get_force_feedback, 1)
        self.force_feedback = None

    def start_control(self):
        """Start the ROS2 node spinning."
        rclpy.spin(self.__node)

    def stop_control(self):
        """Shutdown ROS2.""
        rclpy.shutdown()

    def set_impedance_params(self, impedance_params):
        """Publish the impedance parameters."
        msg = Float64()
        msg.data = impedance_params['speed']
        self.publisher.publish(msg)

    def get_force_feedback(self, msg):
        """Retrieve the force feedback from the message."
        self.force_feedback = msg.data