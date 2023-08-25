from webots_ros2_core.webots_control_node import WebotsControlNode


class MotorController(WebotsControlNode):
    def __init__(self, args):
        super().__init__(args)

    def control_step_webots(self):
        pass

    def control_step_ros2(self):
        pass


def main(args=None):
    controller = MotorController(args)
    controller.run()


if __name__ == '__main__':
    main()