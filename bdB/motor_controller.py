from ros2_control_interfaces.srv import ConfigureControlNode
from webots_ros2_core.math_utils import quat_to_euler
from webots_ros2_core.webots_node import WebotsNode


class MotorController(WebotsNode):
    def __init__(self, args=None):
        super().__init__('webots_ros2_motor_controller', args=args)
        self.use_joint_state = False
        self.use_sensor_state = False

    def use_joint_state(self, arg):
        self.use_joint_state = arg

    def use_sensor_state(self, arg):
        self.use_sensor_state = arg

    async def control_loop(self):
        await self.call_configure_state("")
        while rclpy.ok():
            self.step()

    async def call_configure_state(self, state):
        cli = await self.create_client(ConfigureControlNode, '/webots_ros2_driver_node/configure_state')
        req = ConfigureControlNode.Request()
        req.state = state
        await cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    motor_controller = MotorController()
    rclpy.spin(motor_controller)
    motor_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()