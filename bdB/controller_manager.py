import rclpy
from rcl_interfaces.msg import Parameter
from rclpy.node import Node
from rclpy.duration import Duration

from control_msgs.msg import (
    ConfigureController, 
    ListControllers, 
    LoadController, 
    UnloadController, 
    SwitchController
)


class ControllerManager(Node):
    def __init__(self):
        super().__init__('controller_manager_node')
        self.__configure_controller_service = self.create_service(ConfigureController, 'configure_controller', self.__configure_controller_callback)
        self.__list_controllers_service = self.create_service(ListControllers, 'list_controllers', self.__list_controllers_callback)
        self.__load_controller_service = self.create_service(LoadController, 'load_controller', self.__load_controller_callback)
        self.__unload_controller_service = self.create_service(UnloadController, 'unload_controller', self.__unload_controller_callback)
        self.__switch_controller_service = self.create_service(SwitchController, 'switch_controller', self.__switch_controller_callback)

    def __configure_controller_callback(self, request, response):
        # TODO: Implement the configure_controller callback
        pass

    def __list_controllers_callback(self, request, response):
        # TODO: Implement the list_controllers callback
        pass

    def __load_controller_callback(self, request, response):
        # TODO: Implement the load_controller callback
        pass

    def __unload_controller_callback(self, request, response):
        # TODO: Implement the unload_controller callback
        pass

    def __switch_controller_callback(self, request, response):
        # TODO: Implement the switch_controller callback
        pass


def main():
    rclpy.init()
    controller_manager = ControllerManager()
    rclpy.spin(controller_manager)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
