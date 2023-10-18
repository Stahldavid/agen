import rclpy
from std_msgs.msg import Float64, String
from webots_ros2_msgs.srv import SetString


class MyWebotsROS2Node:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('my_webots_node')
        self.publisher = self.node.create_publisher(Float64, 'my_topic', 1)
        self.subscriber = self.node.create_subscription(String, 'my_topic', self.callback, 1)
        self.service = self.node.create_service(SetString, 'my_service', self.service_callback)
        self.set_parameters = self.node.set_parameters

    def callback(self, msg):
        print('Received message:', msg.data)

    def service_callback(self, request, response):
        response.success = True
        response.message = 'Service called'
        print('Service called with argument:', request.data)
        return response

    def spin(self):
        rclpy.spin(self.node)


if __name__ == '__main__':
    node = MyWebotsROS2Node()
    node.spin()