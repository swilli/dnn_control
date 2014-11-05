import string
from scipy.integrate import odeint

'''

Simulator state:
0 : x
1 : y
2 : z
3 : dx
4 : dy
5 : dz
6 : m

Controller action
0 : T_x
1 : T_y
2 : T_z

'''


class Simulator:


	# Constructor
	def __init__(self, controller, control_frequency, position, velocity, mass):
		self.controller = controller
		self.control_interval = 1.0/control_frequency
		self.state = [position[0], position[1], position[2], velocity[0], velocity[1], velocity[2], mass]


	# Perform the simulation for time seconds
	def run(self, time):
		time_passed = 0
		while time_passed + self.control_interval < time:
			action = self.controller.get_action(self.state)
			self.step(action, time_passed, time_passed + self.control_interval)
			time_passed += self.control_interval


	# Integrate the system from start_time to end_time
	def step(self, action, start_time, end_time):
		result = odeint(self.dynamics, self.state, action, [start_time, end_time])
		self.state = result[1][:]
		print self.state


    # Simulator dynamics from "Control of Hovering Spacecraft Using Altimetry" eq (69)
	def dynamics(self, state, action, time):
		d_dt_state = [0] * 7
		d_dt_state[0] = state[3]
		d_dt_state[1] = state[4]
		d_dt_state[2] = state[5]

		# TODO
		return result

