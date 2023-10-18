import webots_ros2

class MyRobotController:

    def __init__(self):
        self.robot = webots_ros2.robot.Robot()
        # Gather all motors and sensors
        self.motors = [device for device in self.robot.getDevicesByType(webots_ros2.device.Motor)]
        self.position_sensors = [device for device in self.robot.getDevicesByType(webots_ros2.device.PositionSensor)]
        self.torque_feedback_sensors = [device for device in self.robot.getDevicesByType(webots_ros2.device.TorqueFeedbackSensor)]

    def set_impedance(self, stiffness, damping):
        for motor in self.motors:
            motor.available_stiffness = stiffness
            motor.available_damping = damping

    def set_torque_feedback_sampling_period(self, sampling_period):
        for torque_sensor in self.torque_feedback_sensors:
            torque_sensor.torque_feedback_sampling_period = sampling_period
        
    def run(self):
        while self.robot.step() != -1:
            pass

controller = MyRobotController()
controller.set_impedance(500, 50)
controller.set_torque_feedback_sampling_period(10)
controller.run()