import rospy
from std_msgs.msg import String


def variable_impedance_control():
    rospy.init_node('variable_impedance_control', anonymous=True)
    publisher = rospy.Publisher('force_feedback', String, queue_size=10)

    while not rospy.is_shutdown():
        msg = String()
        msg.data = 'Applying variable impedance control'
        publisher.publish(msg)
        rospy.sleep(1)


if __name__ == '__main__':
    try:
        variable_impedance_control()
    except rospy.ROSInterruptException:
        pass