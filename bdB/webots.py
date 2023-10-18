from controller import Robot, AnsiCodes
import math


class MyRobotController(Robot):
    def __init__(self):
        super(MyRobotController, self).__init__()
        self.timeStep = int(self.getBasicTimeStep())
        self.getDevice = self.getDevice
        self.getMotor = self.getMotor
        self.getDistanceSensor = self.getDistanceSensor
        self.step = self.step
        self.print = self.print

    def run(self):
        target = 0
        counter = 0
        while self.step(self.timeStep) != -1:
            self.print(AnsiCodes.CLEAR_SCREEN)
            self.print('Hello, Webots!')
            self.print(f'Time: {self.getTime()}')
            counter += 1
            self.motor = self.getMotor('motor')
            self.motor.setPosition(target)
            if counter == 50:
                target += math.pi / 4
                counter = 0


controller = MyRobotController()
controller.run()