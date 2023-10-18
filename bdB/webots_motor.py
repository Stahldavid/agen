import ctypes
import webots as wb


class WebotsMotor:
    def __init__(self, tag):
        self._tag = tag

    def set_force(self, force):
        wb.wb_motor_set_force(self._tag, ctypes.c_double(force))

    def set_torque(self, torque):
        wb.wb_motor_set_torque(self._tag, ctypes.c_double(torque))

    def get_torque_feedback(self):
        return wb.wb_motor_get_torque_feedback(self._tag)