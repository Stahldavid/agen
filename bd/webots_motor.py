def getTorqueFeedback(self) -> float:
        return self.torque_feedback

    def setForce(self, force: float):
        wb.wb_motor_set_force(self._tag, ctypes.c_double(force))

    def setTorque(self, torque: float):
        wb.wb_motor_set_torque(self._tag, ctypes.c_double(torque))

    def getType(self) -> int:
        return wb.wb_motor_get_type(self._tag)

    force = property(fset=setForce)
    torque = property(fset=setTorque)