import ctypes
import wb


class WebotsMotorWrapper:
    def __init__(self, tag):
        self._tag = tag

    def getTorqueFeedback(self) -> float:
        return wb.wb_motor_get_torque_feedback(self._tag)

    def setForce(self, force: float):
        wb.wb_motor_set_force(self._tag, ctypes.c_double(force))

    def setTorque(self, torque: float):
        wb.wb_motor_set_torque(self._tag, ctypes.c_double(torque))

    # Other motor wrapper methods...