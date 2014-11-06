import string
import math
import constants
import numpy
import sys
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
	def __init__(self, asteroid_a_semi_axis, asteroid_b_semi_axis, asteroid_c_semi_axis, asteroid_density, asteroid_theta, asteroid_omega_zero, spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse, sensor_simulator, controller, control_frequency):
		self.sensor_simulator = sensor_simulator
		self.controller = controller
		self.control_interval = 1.0/control_frequency

		self.cos_theta = math.cos(asteroid_theta)
		self.cos_theta_pow2 = self.cos_theta**2

		self.sin_theta = math.sin(asteroid_theta)
		self.sin_theta_pow2 = self.sin_theta**2

		self.omega_zero = asteroid_omega_zero
		self.omega_zero_mul2 = self.omega_zero * 2
		self.omega_zero_pow2 = self.omega_zero**2

		sigma = (asteroid_a_semi_axis**2 - asteroid_c_semi_axis**2)/(asteroid_b_semi_axis**2 + asteroid_c_semi_axis**2)
		self.omega_n = sigma * self.omega_zero * self.cos_theta

		self.spacecraft_specific_impulse = spacecraft_specific_impulse
		self.spacecraft_state = numpy.array([spacecraft_position[0], spacecraft_position[1], spacecraft_position[2], spacecraft_velocity[0], spacecraft_velocity[1], spacecraft_velocity[2], spacecraft_mass])
		self.earth_acceleration_mul_specific_impulse = constants.EARTH_ACCELERATION * self.spacecraft_specific_impulse


	# Perform the simulation for time seconds
	def run(self, time, collect_positions=False):
		control_interval = self.control_interval
		iterations = int(time/self.control_interval)
		positions = []
		
		if collect_positions == True:
			positions = numpy.empty([iterations,3])

		for i in range(iterations):
			sensor_data = self.sensor_simulator.simulate(self.spacecraft_state)
			action = self.controller.get_action(sensor_data)
			pertubations_acceleration = self.simulate_pertubations()

			if collect_positions == True:
				positions[i][:] = self.spacecraft_state[0:3]
			
			self.simulate_dynamics(pertubations_acceleration, action, i*control_interval, (i+1)*control_interval)

		return positions


	# Integrate the system from start_time to end_time
	def simulate_dynamics(self, pertubations_acceleration, action, start_time, end_time):
		result = odeint(self.dynamics, self.spacecraft_state, [start_time, end_time], (pertubations_acceleration,action))
		self.spacecraft_state = result[1][:]


    # Simulator dynamics from "Control of Hovering Spacecraft Using Altimetry" eq (69) and "Robust Spacecraft Hovering Near Small Bodies in Environments with Unknown Dynamics using Reinforcement Learning" eq (6)
	def dynamics(self, state, time, pertubations_acceleration, action):
		cos_theta = self.cos_theta
		cos_theta_pow2 = self.cos_theta_pow2
		sin_theta = self.sin_theta
		sin_theta_pow2 = self.sin_theta_pow2
		omega_zero_mul2 = self.omega_zero_mul2
		omega_zero_pow2 = self.omega_zero_pow2

		sin_omega_n_t = math.sin(self.omega_n*time)
		sin_omega_n_t_pow2 = sin_omega_n_t**2
		cos_omega_n_t = math.cos(self.omega_n*time)
		cos_omega_n_t_pow2 = cos_omega_n_t**2

		gravity_acceleration = self.simulate_gravity(state[0:3])
		action_acceleration = [action[0]/state[6], action[1]/state[6], action[2]/state[6]]

		d_dt_state = [0,0,0,0,0,0,0]
		d_dt_state[0] = state[3]
		d_dt_state[1] = state[4]
		d_dt_state[2] = state[5]

		d_dt_state[3] = pertubations_acceleration[0] + gravity_acceleration[0] + action_acceleration[0] - omega_zero_mul2*(-state[4]*cos_theta + state[5]*sin_theta*sin_omega_n_t) - omega_zero_pow2*(-state[0]*(sin_theta_pow2*sin_omega_n_t_pow2 + cos_theta_pow2) + state[1]*sin_theta_pow2*sin_omega_n_t*cos_omega_n_t + state[2]*sin_theta*cos_theta*cos_omega_n_t)
		d_dt_state[4] = pertubations_acceleration[1] + gravity_acceleration[1] + action_acceleration[1] - omega_zero_mul2*(state[3]*cos_theta - state[5]*sin_theta*cos_omega_n_t) - omega_zero_pow2*(state[0]*sin_theta_pow2*sin_omega_n_t*cos_omega_n_t - state[1]*(sin_theta_pow2*cos_omega_n_t_pow2 + cos_theta_pow2) + state[2]*(sin_theta*cos_theta*sin_omega_n_t))
		d_dt_state[5] = pertubations_acceleration[2] + gravity_acceleration[2] + action_acceleration[2] - omega_zero_mul2*(-state[3]*sin_theta*sin_omega_n_t + state[4]*sin_theta*cos_omega_n_t) - omega_zero_pow2*(state[0]*sin_theta*cos_theta*cos_omega_n_t + state[1]*sin_theta*cos_theta*sin_omega_n_t - state[2]*sin_theta_pow2)
		
		# d/dt m
		d_dt_state[6] = math.sqrt(action[0]**2 + action[1]**2 + action[2]**2) / self.earth_acceleration_mul_specific_impulse

		return d_dt_state

	# External pertubations acceleration
	def simulate_pertubations(self):
		mean = 1e-4
		variance = 1e-5 
		spacecraft_mass = self.spacecraft_state[6]
		return [0,0,0]
		#return[spacecraft_mass*numpy.random.normal(mean, variance), spacecraft_mass*numpy.random.normal(mean, variance), spacecraft_mass*numpy.random.normal(mean, variance)]


	# Gravity acceleration at position
	def simulate_gravity(self, position):
		return [0,0,0]
