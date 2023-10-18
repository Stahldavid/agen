from robot_driver import RobotDriver
from obstacle_avoider import ObstacleAvoider
from control import Control
from motor import Motor


def main():
    robot_driver = RobotDriver()
    obstacle_avoider = ObstacleAvoider()
    control = Control()
    motor = Motor()

    # Perform variable impedance control
    robot_driver.step()
    obstacle_avoider.step()
    control.set_parameter('param1', 'value1')
    motor.set_force(10)


if __name__ == '__main__':
    main()