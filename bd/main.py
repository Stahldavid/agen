import rclpy
from webots_interface import WebotsInterface
from ros2_interface import Ros2Interface
from ros2_control import Ros2Control

class VariableImpedanceController:
    """A class to control the interaction between Webots, ROS2 and the impedance controller."

    def __init__(self):
        """Initialize the controller with interfaces to Webots, ROS2, and the impedance controller."
        self.webots_interface = WebotsInterface()
        self.ros2_interface = Ros2Interface()
        self.ros2_control = Ros2Control()

    def start_controller(self):
        """Start the ROS2 interface and the impedance controller."
        self.ros2_interface.start_interface()
        self.ros2_control.start_control()

    def run(self):
        """Run the controller, sending impedance commands and setting impedance parameters."
        while rclpy.ok():
            self.ros2_interface.send_impedance_command({'speed': self.webots_interface.get_force_feedback()})
            self.ros2_control.set_impedance_params({'speed': self.ros2_interface.force_feedback})

    def stop_controller(self):
        """Stop the ROS2 interface and the impedance controller."
        self.ros2_interface.stop_interface()
        self.ros2_control.stop_control()


if __name__ == '__main__':
    controller = VariableImpedanceController()
    controller.start_controller()

    try:
        controller.run()
    except KeyboardInterrupt:
        controller.stop_controller()