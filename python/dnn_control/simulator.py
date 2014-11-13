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
    def __init__(self, asteroid, spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse, sensor_simulator, spacecraft_controller, control_frequency):
        from numpy import array
        from constants import EARTH_ACCELERATION

        self.sensor_simulator = sensor_simulator
        self.spacecraft_controller = spacecraft_controller
        self.control_interval = 1.0 / control_frequency

        self.asteroid = asteroid

        self.spacecraft_specific_impulse = float(spacecraft_specific_impulse)
        self.spacecraft_state = array([float(spacecraft_position[0]), float(spacecraft_position[1]), float(spacecraft_position[2]), float(
            spacecraft_velocity[0]), float(spacecraft_velocity[1]), float(spacecraft_velocity[2]), float(spacecraft_mass)])

        self.earth_acceleration_mul_spacecraft_specific_impulse = EARTH_ACCELERATION * \
            self.spacecraft_specific_impulse

    # Perform the simulation for time seconds
    def run(self, time, collect_positions=False):
        from numpy import empty

        control_interval = self.control_interval
        iterations = int(time / self.control_interval)
        positions = []

        if collect_positions == True:
            positions = empty([iterations, 3])

        for i in range(iterations):
            sensor_data = self.sensor_simulator.simulate(self.spacecraft_state)
            thrust = self.spacecraft_controller.get_thrust(sensor_data)
            pertubations_acceleration = self.simulate_pertubations()

            if collect_positions == True:
                positions[i][:] = self.spacecraft_state[0:3]

            self.simulate_dynamics(
                pertubations_acceleration, thrust, i * control_interval, (i + 1) * control_interval)

        return positions

    # Integrate the system from start_time to end_time
    def simulate_dynamics(self, pertubations_acceleration, thrust, start_time, end_time):
        from scipy.integrate import odeint

        result = odeint(self.dynamics, self.spacecraft_state, [
                        start_time, end_time], (pertubations_acceleration, thrust))
        self.spacecraft_state = result[1][:]

    # Simulator dynamics from "Control of Hovering Spacecraft Using Altimetry"
    # eq (69) and "Robust Spacecraft Hovering Near Small Bodies in
    # Environments with Unknown Dynamics using Reinforcement Learning" eq (6)
    def dynamics(self, state, time, pertubations_acceleration, thrust):
        from utility import cross_product
        from math import sqrt

        position = state[0:3]
        velocity = state[3:6]
        mass = state[6]

        gravity = self.simulate_gravity(state[0:3])
        gravity_acceleration = [val / mass for val in gravity]
        thrust_acceleration = [val / mass for val in thrust]

        angular_velocity = self.asteroid.angular_velocity_at_time(time)
        angular_velocity_mul2 = [2.0 * val for val in angular_velocity]
        angular_acceleration = self.asteroid.angular_acceleration_at_time(time)

        centrifugal_acceleration = cross_product(
            angular_velocity, cross_product(angular_velocity, position))
        coriolis_acceleration = cross_product(angular_velocity_mul2, velocity)
        euler_acceleration = cross_product(angular_acceleration, position)

        d_dt_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        # d/dt r
        d_dt_state[0] = velocity[0]
        d_dt_state[1] = velocity[1]
        d_dt_state[2] = velocity[2]

        # d/dt v
        d_dt_state[3] = pertubations_acceleration[0] + gravity_acceleration[0] + thrust_acceleration[
            0] - coriolis_acceleration[0] - euler_acceleration[0] - centrifugal_acceleration[0]
        d_dt_state[4] = pertubations_acceleration[1] + gravity_acceleration[1] + thrust_acceleration[
            1] - coriolis_acceleration[1] - euler_acceleration[1] - centrifugal_acceleration[1]
        d_dt_state[5] = pertubations_acceleration[2] + gravity_acceleration[2] + thrust_acceleration[
            2] - coriolis_acceleration[2] - euler_acceleration[2] - centrifugal_acceleration[2]

        # d/dt m
        d_dt_state[6] = sqrt(thrust[0] ** 2 + thrust[1] ** 2 + thrust[2]
                             ** 2) / self.earth_acceleration_mul_spacecraft_specific_impulse
        return d_dt_state

    # External pertubations acceleration
    def simulate_pertubations(self):
        from numpy import random

        mean = 0.0
        variance = 1e-6
        spacecraft_mass = self.spacecraft_state[6]
        return[spacecraft_mass * random.normal(mean, variance), spacecraft_mass * random.normal(mean, variance), spacecraft_mass * random.normal(mean, variance)]

    # Gravity acceleration at position
    def simulate_gravity(self, position):
        return self.asteroid.gravity_at_position(position)
