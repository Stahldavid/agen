import rclpy

from variable_impedance_control import VariableImpedanceControl
from ros2_control_interface import ControlInterfaceCommand
from webots_interface import WebotsInterface
from ros2_webots_bridge import ROS2WebotsBridge


def main(args=None):
    rclpy.init(args=args)

    # Create instances of the components
    control = VariableImpedanceControl()
    interface = ControlInterfaceCommand()
    webots = WebotsInterface()
    bridge = ROS2WebotsBridge()

    # Start the components
    rclpy.spin(control.subscriber)
    rclpy.spin(interface)
    bridge.run()

    # Clean up
    control.subscriber.destroy_node()
    bridge.node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()