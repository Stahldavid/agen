import ctypes


class Motor:
    def __init__(self, tag):
        self._tag = tag

    def set_force(self, force):
        pass

    def set_torque(self, torque):
        pass

    def get_type(self):
        pass

    def get_brake(self):
        pass

    def get_position_sensor(self):
        pass


class PositionSensor:
    def __init__(self, tag):
        self._tag = tag

    def get_value(self):
        pass


class Brake:
    def __init__(self, tag):
        self._tag = tag

    def enable(self):
        pass

    def disable(self):
        pass


class ForceFeedback:
    def __init__(self, tag):
        self._tag = tag

    def enable(self):
        pass

    def disable(self):
        pass