from vehicle import Driver

class WebotsInterface:
    "Class to provide an interface to the Webots vehicle driver."

    def __init__(self):
        "Initialize the interface and set the steering angle to zero."
        self.driver = Driver()
        self.driver.setSteeringAngle(0.0)

    def start_interface(self):
        "Start the Webots driver."
        while self.driver.step() != -1:
            pass

    def stop_interface(self):
        "Stop the Webots driver."
        self.driver.stop()

    def get_force_feedback(self):
        """
        Get the force feedback from the 'front' sensor.
        Replace 'front' with the actual sensor name.
        """
        return self.driver.getDevice('front').getValue()

    def apply_impedance_params(self, impedance_params):
        """
        Apply the impedance parameters by setting the cruising speed and brake intensity.
        """
        speed = impedance_params['speed']
        self.driver.setCruisingSpeed(speed)
        speed_diff = self.driver.getCurrentSpeed() - speed
        if speed_diff > 0:
            self.driver.setBrakeIntensity(min(speed_diff / speed, 1))
        else:
            self.driver.setBrakeIntensity(0)
