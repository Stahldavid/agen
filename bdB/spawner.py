def getTorqueFeedback(self):
    return self.torque_feedback

def setForce(self, force):
    wb.wb_motor_set_force(self._tag, ctypes.c_double(force))

def setTorque(self, torque):
    wb.wb_motor_set_torque(self._tag, ctypes.c_double(torque))

def getType(self):
    return wb.wb_motor_get_type(self._tag)

def brake(self):
    from .brake import Brake
    tag = wb.wb_motor_get_brake(self._tag)
    return None if tag == 0 else Brake(tag)

@property
def position_sensor(self):
    from .position_sensor import PositionSensor
    tag = wb.wb_motor_get_position_sensor(self._tag)
    return None if tag == 0 else PositionSensor(tag)