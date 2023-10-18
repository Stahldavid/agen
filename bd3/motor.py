def getTorqueFeedback(self) -> float:
    return self.torque_feedback


def setForce(self, force: float):
    wb.wb_motor_set_force(self._tag, ctypes.c_double(force))

def setTorque(self, torque: float):
    wb.wb_motor_set_torque(self._tag, ctypes.c_double(torque))

def getType(self) -> int:
    return wb.wb_motor_get_type(self._tag)


def getBrake(self):
    return self.brake


def getPositionSensor(self):
    return self.position_sensor


@property
def brake(self):
    from .brake import Brake
    tag = wb.wb_motor_get_brake(self._tag)
    return None if tag == 0 else Brake(tag)


@property
def position_sensor(self):
    from .position_sensor import PositionSensor
    tag = wb.wb_motor_get_position_sensor(self._tag)
    return None if tag == 0 else PositionSensor(tag)


@property
def force_feedback_sampling_period(self) -> int:
    return wb.wb_motor_get_force_feedback_sampling_period(self._tag)


@force_feedback_sampling_period.setter
def force_feedback_sampling_period(self, sampling_period):
    wb.wb_motor_enable_force_feedback(self._tag, sampling_period)


@property
def torque_feedback_sampling_period(self) -> int:
    return wb.wb_motor_get_torque_feedback_sampling_period(self._tag)


@torque_feedback_sampling_period.setter
def torque_feedback_sampling_period(self, sampling_period):
    wb.wb_motor_enable_torque_feedback(self._tag, sampling_period)


@property
def max_position(self) -> float:
    return wb.wb_motor_get_max_position(self._tag)


@property
def min_position(self) -> float:
    return wb.wb_motor_get_min_position(self._tag)


@property
def max_velocity(self) -> float:
    return wb.wb_motor_get_max_velocity(self._tag)


@property
def target_position(self) -> float:
    return wb.wb_motor_get_target_position(self._tag)


@target_position.setter
def target_position(self, position: float):
    wb.wb_motor_set_position(self._tag, ctypes.c_double(position))


@property
def target_velocity(self) -> float:
    return wb.wb_motor_get_velocity(self._tag)


@target_velocity.setter
def target_velocity(self, velocity: float):
    wb.wb_motor_set_velocity(self._tag, ctypes.c_double(velocity))


@property
def available_force(self) -> float:
    return wb.wb_motor_get_available_force(self._tag)


@available_force.setter
def available_force(self, force: float):
    wb.wb_motor_set_available_force(self._tag, ctypes.c_double(force))


@property
def max_force(self) -> float:
    return wb.wb_motor_get_max_force(self._tag)


@property
def available_torque(self) -> float:
    return wb.wb_motor_get_available_torque(self._tag)


@available_torque.setter
def available_torque(self, torque: float):
    wb.wb_motor_set_available_torque(self._tag, ctypes.c_double(torque))


@property
def max_torque(self) -> float:
    return wb.wb_motor_get_max_torque(self._tag)


@property
def target_acceleration(self) -> float:
    return wb.wb_motor_get_acceleration(self._tag)


@target_acceleration.setter
def target_acceleration(self, acceleration: float):
    wb.wb_motor_set_acceleration(self._tag, ctypes.c_double(acceleration))


force = property(fset=setForce)
torque = property(fset=setTorque)


@property
def multiplier(self) -> float:
    return wb.wb_motor_get_multiplier(self._tag)


@property
def force_feedback(self) -> float:
    return wb.wb_motor_get_force_feedback(self._tag)


@property
def torque_feedback(self) -> float:
    return wb.wb_motor_get_torque_feedback(self._tag)


@property
def type(self) -> int:
    return wb.wb_motor_get_type(self._tag)