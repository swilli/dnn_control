import string
import math
import numpy
import sys
from scipy.integrate import odeint
import constants
import ellipsoidgravityfield

'''

Simulator state:
0 : x
1 : y
2 : z
3 : dx
4 : dy
5 : dz
6 : m

spacecraft_controller thrust
0 : T_x
1 : T_y
2 : T_z

'''


class Simulator:

	# Constructor
	# requires asteroid_semi_axis_a > asteroid_semi_axis_b > asteroid_semi_axis_c
	def __init__(self, asteroid_semi_axis_a, asteroid_semi_axis_b, asteroid_semi_axis_c, asteroid_density, asteroid_theta, asteroid_omega_zero, spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse, sensor_simulator, spacecraft_controller, control_frequency):
		self.sensor_simulator = sensor_simulator
		self.spacecraft_controller = spacecraft_controller
		self.control_interval = 1.0/control_frequency

		self.asteroid_density = float(asteroid_density)
		self.asteroid_cos_theta = math.cos(asteroid_theta)
		self.asteroid_cos_theta_pow2 = self.asteroid_cos_theta**2

		self.asteroid_sin_theta = math.sin(asteroid_theta)
		self.asteroid_sin_theta_pow2 = self.asteroid_sin_theta**2

		self.asteroid_omega_zero = float(asteroid_omega_zero)
		self.asteroid_omega_zero_mul2 = self.asteroid_omega_zero * 2
		self.asteroid_omega_zero_pow2 = self.asteroid_omega_zero**2

		self.asteroid_semi_axis_a = float(asteroid_semi_axis_a)
		self.asteroid_semi_axis_a_pow2 = self.asteroid_semi_axis_a**2
		self.asteroid_semi_axis_b = float(asteroid_semi_axis_b)
		self.asteroid_semi_axis_b_pow2 = self.asteroid_semi_axis_b**2
		self.asteroid_semi_axis_c = float(asteroid_semi_axis_c)
		self.asteroid_semi_axis_c_pow2 = self.asteroid_semi_axis_c**2

		sigma = (self.asteroid_semi_axis_a_pow2 - self.asteroid_semi_axis_c_pow2)/(self.asteroid_semi_axis_b_pow2 + self.asteroid_semi_axis_c_pow2)
		self.asteroid_omega_n = sigma * self.asteroid_omega_zero * self.asteroid_cos_theta

		self.spacecraft_specific_impulse = float(spacecraft_specific_impulse)
		self.spacecraft_state = numpy.array([float(spacecraft_position[0]), float(spacecraft_position[1]), float(spacecraft_position[2]), float(spacecraft_velocity[0]), float(spacecraft_velocity[1]), float(spacecraft_velocity[2]), float(spacecraft_mass)])

		self.earth_acceleration_mul_spacecraft_specific_impulse = constants.EARTH_ACCELERATION * self.spacecraft_specific_impulse


	# Perform the simulation for time seconds
	def run(self, time, collect_positions=False):
		control_interval = self.control_interval
		iterations = int(time/self.control_interval)
		positions = []
		
		if collect_positions == True:
			positions = numpy.empty([iterations,3])

		for i in range(iterations):
			sensor_data = self.sensor_simulator.simulate(self.spacecraft_state)
			thrust = self.spacecraft_controller.get_thrust(sensor_data)
			pertubations_acceleration = self.simulate_pertubations()

			if collect_positions == True:
				positions[i][:] = self.spacecraft_state[0:3]
			
			self.simulate_dynamics(pertubations_acceleration, thrust, i*control_interval, (i+1)*control_interval)

		return positions


	# Integrate the system from start_time to end_time
	def simulate_dynamics(self, pertubations_acceleration, thrust, start_time, end_time):
		result = odeint(self.dynamics, self.spacecraft_state, [start_time, end_time], (pertubations_acceleration,thrust))
		self.spacecraft_state = result[1][:]


    # Simulator dynamics from "Control of Hovering Spacecraft Using Altimetry" eq (69) and "Robust Spacecraft Hovering Near Small Bodies in Environments with Unknown Dynamics using Reinforcement Learning" eq (6)
	def dynamics(self, state, time, pertubations_acceleration, thrust):
		cos_theta = self.asteroid_cos_theta
		cos_theta_pow2 = self.asteroid_cos_theta_pow2
		sin_theta = self.asteroid_sin_theta
		sin_theta_pow2 = self.asteroid_sin_theta_pow2
		omega_zero_mul2 = self.asteroid_omega_zero_mul2
		omega_zero_pow2 = self.asteroid_omega_zero_pow2

		sin_omega_n_t = math.sin(self.asteroid_omega_n*time)
		sin_omega_n_t_pow2 = sin_omega_n_t**2
		cos_omega_n_t = math.cos(self.asteroid_omega_n*time)
		cos_omega_n_t_pow2 = cos_omega_n_t**2

		gravity_acceleration = self.simulate_gravity(state[0:3])
		thrust_acceleration = [thrust[0]/state[6], thrust[1]/state[6], thrust[2]/state[6]]

		d_dt_state = [0, 0, 0, 0, 0, 0, 0]
		d_dt_state[0] = state[3]
		d_dt_state[1] = state[4]
		d_dt_state[2] = state[5]

		d_dt_state[3] = pertubations_acceleration[0] + gravity_acceleration[0] + thrust_acceleration[0] - omega_zero_mul2*(-state[4]*cos_theta + state[5]*sin_theta*sin_omega_n_t) - omega_zero_pow2*(-state[0]*(sin_theta_pow2*sin_omega_n_t_pow2 + cos_theta_pow2) + state[1]*sin_theta_pow2*sin_omega_n_t*cos_omega_n_t + state[2]*sin_theta*cos_theta*cos_omega_n_t)
		d_dt_state[4] = pertubations_acceleration[1] + gravity_acceleration[1] + thrust_acceleration[1] - omega_zero_mul2*(state[3]*cos_theta - state[5]*sin_theta*cos_omega_n_t) - omega_zero_pow2*(state[0]*sin_theta_pow2*sin_omega_n_t*cos_omega_n_t - state[1]*(sin_theta_pow2*cos_omega_n_t_pow2 + cos_theta_pow2) + state[2]*(sin_theta*cos_theta*sin_omega_n_t))
		d_dt_state[5] = pertubations_acceleration[2] + gravity_acceleration[2] + thrust_acceleration[2] - omega_zero_mul2*(-state[3]*sin_theta*sin_omega_n_t + state[4]*sin_theta*cos_omega_n_t) - omega_zero_pow2*(state[0]*sin_theta*cos_theta*cos_omega_n_t + state[1]*sin_theta*cos_theta*sin_omega_n_t - state[2]*sin_theta_pow2)
		
		# d/dt m
		d_dt_state[6] = math.sqrt(thrust[0]**2 + thrust[1]**2 + thrust[2]**2) / self.earth_acceleration_mul_spacecraft_specific_impulse

		return d_dt_state

	# External pertubations acceleration
	def simulate_pertubations(self):
		mean = 0
		variance = 1e-6 
		spacecraft_mass = self.spacecraft_state[6]
		return[spacecraft_mass*numpy.random.normal(mean, variance), spacecraft_mass*numpy.random.normal(mean, variance), spacecraft_mass*numpy.random.normal(mean, variance)]


	# Gravity acceleration at position 
	# Ported from "Ellipsoid_Gravity_Field.m" written by Dario Cersosimo, October 16, 2009.
	def simulate_gravity(self, position):
		return ellipsoidgravityfield.ellipsoid_gravity_field(position , self.asteroid_semi_axis_a, self.asteroid_semi_axis_b, self.asteroid_semi_axis_c, self.asteroid_density)