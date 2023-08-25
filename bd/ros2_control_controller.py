import rclpy
from rclpy.node import Node
from ros2_control_msgs.msg import JointControllerState


class Ros2ControlController(Node):
    def __init__(self):
        super().__init__('ros2_control_controller')

        self.__state_publisher = self.create_publisher(JointControllerState, 'state', 1)
        self.__timer = self.create_timer(1.0, self.__timer_callback)

        self.__joint_state = JointControllerState()
        self.__joint_state.header.frame_id = 'robot'
        self.__joint_state.joint_name = 'joint1'
        self.__joint_state.set_point = 0.0

    def __timer_callback(self):
        self.__state_publisher.publish(self.__joint_state)


def main(args=None):
    rclpy.init(args=args)
    controller = Ros2ControlController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()