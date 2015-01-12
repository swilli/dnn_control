class FullStateController:

    def __init__(self, target_position, control_interval):
        self._target_position = target_position
        self._control_interval = control_interval
        self._previous_error = [0.0, 0.0, 0.0]
        self._integral = [0.0, 0.0, 0.0]
        self._constant_proportional = 1.0
        self._constant_derivative = 1.5
        self._constant_integral = 0.0

    def get_thrust_for_sensor_data(self, sensor_data):
        current_position = sensor_data[0:3]
        thrust = [0.0, 0.0, 0.0]
        for i in [0, 1, 2]:
            error = self._target_position[i] - current_position[i]
            self._integral[i] += error * self._control_interval
            derivative = (error - self._previous_error[i]) / self._control_interval
            thrust[i] = self._constant_proportional * error\
                        + self._constant_derivative * derivative \
                        + self._constant_integral * self._integral[i]
            self._previous_error[i] = error

        return thrust
