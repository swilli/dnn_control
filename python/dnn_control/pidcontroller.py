class PIDController:

    def __init__(self, target_position, time_interval):
        self.target_position = target_position
        self.time_interval = time_interval
        self.previous_error = [0.0, 0.0, 0.0]
        self.integral = [0.0, 0.0, 0.0]
        self.coef_proportional = [30.0, 30.0, 30.0]
        self.coef_integral = [0.0, 0.0, 0.0]
        self.coef_derivative = [30.0, 30.0, 30.0]

    def get_thrust(self, sensor_data):
        '''

        target_position = self.target_position
        current_position = state[0:3]
        thrust = [0.0, 0.0, 0.0]
        error = [0.0, 0.0, 0.0]
        for i in range(3):
            error[i] = target_position[i] - current_position[i]
            self.integral[i] = self.integral[i] + error[i] * self.time_interval
            derivative = (
                error[i] - self.previous_error[i]) / self.time_interval
            self.previous_error[i] = error[i]
            thrust[i] = self.coef_proportional[i] * error[i] + self.coef_integral[i] * \
                self.integral[i] + self.coef_derivative[i] * derivative

        '''

        return [0.0, 0.0, 0.0]
