from controller import Robot, AnsiCodes
import math


class Controller(Robot):
    def __init__(self):
        super(Controller, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())
        self.motor = self.getDevice('motor')
        self.motor.enableTorqueFeedback(self.timeStep)
        self.batterySensorEnable(self.timeStep)

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
            print(f'Force feedback = {self.motor.getTorqueFeedback()}')
            print(f'Battery level  = {self.batterySensorGetValue()}')

if __name__ == '__main__':
    controller = Controller()
    controller.run()