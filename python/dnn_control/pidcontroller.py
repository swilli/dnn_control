class PIDController:

	def __init__(self, target_position, time_interval):
		self.target_position = target_position
		self.time_interval = time_interval
		self.previous_error = [0.0,0.0,0.0]
		self.integral = [0.0,0.0,0.0]
		self.coef_proportional = 0.0001
		self.coef_integral = 0.0
		self.coef_derivative = 0.0001

	def get_thrust(self, state):
		target_position = self.target_position
		current_position = state[0:3]
		thrust = [0.0,0.0,0.0]
		error = [0.0,0.0,0.0]
		for i in range(3):
			error[i] = current_position[i]-target_position[i]
			self.integral[i] = self.integral[i] + error[i]*self.time_interval
			derivative = (error[i] - self.previous_error[i])/self.time_interval
			self.previous_error[i] = error[i]
			thrust[i] = self.coef_proportional*error[i] + self.coef_integral*self.integral[i] + self.coef_derivative*derivative

		print("error: {0}".format(error))
		return thrust