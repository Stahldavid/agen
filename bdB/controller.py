from controller import Robot, AnsiCodes
import math

# Improved class incorporating best practices and including the feedback control mechanism
class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = self.getBasicTimeStep()
        self.motor = self.getDevice('motor')
        self.motor.enablePosition(True)
        self.motor.enableForceFeedback(self.timeStep)
        self.motor.setTorque(5)  # Set a suitable torque
        self.batterySensor = self.getDevice('battery Sensor')
        self.batterySensor.enable(self.timeStep)

    def run(self):
        target = 0
        counter = 0
        while self.step(self.timeStep) != -1:
            self.motor.setPosition(target)
            if counter == 50:
                target += math.pi / 4
                counter = 0
            counter += 1
            print(AnsiCodes.CLEAR_SCREEN)
            print(f'Force feedback = {self.motor.getForceFeedback()}')
            print(f'Battery level  = {self.batterySensor.getValue()}')

def main(args=None):
    controller = Controller()
    controller.run()

if __name__ == '__main__':
    main()