from webots_ros2 import WebotsInterface
import argparse
import rospy
from controller import Robot
from rosgraph_msgs.msg import Clock


class WebotsROS2Interface:
    def __init__(self):
        self.interface = WebotsInterface()

    def connect(self):
        # Connect to Webots
        pass

    def disconnect(self):
        # Disconnect from Webots
        pass

    def update(self):
        # Update interface between Webots and ROS2
        pass

    def set_force_feedback(self, force):
        # Set force feedback in Webots
        pass

    def set_torque_feedback(self, torque):
        # Set torque feedback in Webots
        pass


def main(args=None):
    interface = WebotsROS2Interface()
    interface.connect()

    while True:
        interface.update()

        # Variable impedance control logic
        pass

    interface.disconnect()


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--node-name', dest='nodeName', default='ur_driver', help='Specifies the name of the node.')
    arguments, unknown = parser.parse_known_args()

    rospy.init_node(arguments.nodeName, disable_signals=True)

    jointPrefix = rospy.get_param('prefix', '')
    if jointPrefix:
        print('Setting prefix to %s' % jointPrefix)

    robot = Robot()
    nodeName = arguments.nodeName + '/' if arguments.nodeName != 'ur_driver' else ''
    jointStatePublisher = JointStatePublisher(robot, jointPrefix, nodeName)
    trajectoryFollower = TrajectoryFollower(robot, jointStatePublisher, jointPrefix, nodeName)
    trajectoryFollower.start()

    # we want to use simulation time for ROS
    clockPublisher = rospy.Publisher('clock', Clock, queue_size=1)
    if not rospy.get_param('use_sim_time', False):
        rospy.logwarn('use_sim_time is not set!')

    timestep = int(robot.getBasicTimeStep())

    while robot.step(timestep) != -1 and not rospy.is_shutdown():
        jointStatePublisher.publish()
        trajectoryFollower.update()
        # publish simulation clock
        msg = Clock()
        time = robot.getTime()
        msg.clock.secs = int(time)
        # round prevents precision issues that can cause problems with ROS timers
        msg.clock.nsecs = int(round(1000 * (time - msg.clock.secs)) * 1.0e+6)
        clockPublisher.publish(msg)