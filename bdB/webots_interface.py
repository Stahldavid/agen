import rclpy
from sensor_msgs.msg import JointState


class WebotsInterface:
    def __init__(self):
        self.joint_state_publisher = rclpy.create_publisher(JointState, 'joint_states', 10)

    def update_joint_states(self):
        # Update joint states from Webots
        joint_states = JointState()
        # Populate joint_states with values from Webots
        self.joint_state_publisher.publish(joint_states)