class Simulator:

    def __init__(self, asteroid, spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse,
                 sensor_simulator, spacecraft_controller, control_frequency):
        """
        Constructor. The spacecraft state is defined as follows:
        0 : position x
        1 : position y
        2 : position z
        3 : velocity x
        4 : velocity y
        5 : velocity z
        6 : mass

        The simulator expects the controller to return the following vector when calling get_thrust():
        0 : thrust x
        1 : thrust y
        2 : thrust z
        """

        from numpy import array
        from constants import EARTH_ACCELERATION

        self.sensor_simulator = sensor_simulator
        self.spacecraft_controller = spacecraft_controller
        self.control_interval = 1.0 / control_frequency

        self.asteroid = asteroid

        self.spacecraft_specific_impulse = float(spacecraft_specific_impulse)
        self.spacecraft_state = array([float(spacecraft_position[0]),
                                       float(spacecraft_position[1]),
                                       float(spacecraft_position[2]),
                                       float(spacecraft_velocity[0]),
                                       float(spacecraft_velocity[1]),
                                       float(spacecraft_velocity[2]),
                                       float(spacecraft_mass)])

        self._earth_acceleration_mul_spacecraft_specific_impulse = EARTH_ACCELERATION * \
                                                                   self.spacecraft_specific_impulse


    def __str__(self):
        result = ["Simulator:"]
        keys = sorted([key for key in self.__dict__])
        for key in keys:
            result.append("{key}='{value}'".format(key=key, value=self.__dict__[key]))

        return "\n ".join(result)

    # Perform the simulation for time seconds
    def run(self, time, collect_data=False):
        from numpy import empty

        control_interval = self.control_interval
        iterations = int(time / self.control_interval)
        positions = []
        velocities = []
        heights = []
        velocities_vertical = []
        velocities_remaining = []

        if collect_data:
            positions = empty([iterations, 3])
            velocities = empty([iterations, 3])
            heights = empty([iterations, 3])
            velocities_vertical = empty([iterations, 3])
            velocities_remaining = empty([iterations, 3])

        for i in range(iterations):
            start_time = i * control_interval
            end_time = start_time + control_interval

            # Get new perturbations
            perturbations_acceleration = self.simulate_perturbations()

            # Simulate sensor data for current spacecraft state
            sensor_data, height, velocity_vertical, velocity_remaining = self.sensor_simulator.simulate(self.spacecraft_state, perturbations_acceleration, start_time)

            # Get thrust from controller
            thrust = self.spacecraft_controller.get_thrust(sensor_data)

            if collect_data:
                positions[i][:] = self.spacecraft_state[0:3]
                velocities[i][:] = self.spacecraft_state[3:6]
                heights[i][:] = height
                velocities_vertical[i][:] = velocity_vertical
                velocities_remaining[i][:] = velocity_remaining

            # Simulate dynamics with current perturbations and thrust
            self.simulate_dynamics(perturbations_acceleration, thrust, start_time, end_time)

        return positions, velocities, heights, velocities_vertical, velocities_remaining

    # Integrate the system from start_time to end_time
    def simulate_dynamics(self, perturbations_acceleration, thrust, start_time, end_time):
        from scipy.integrate import odeint
        from math import isnan, isinf

        result = odeint(self.dynamics, self.spacecraft_state, [start_time, end_time],
                        (perturbations_acceleration, thrust))
        self.spacecraft_state = result[1][:]

        for val in self.spacecraft_state:
            if isnan(val) or isinf(val):
                raise Exception(self)
                exit()

    # Simulator dynamics from eq (1) in paper "Control of Hovering Spacecraft Using Altimetry" by Sawai et al.
    def dynamics(self, state, time, perturbations_acceleration, thrust):
        from utility import cross_product
        from math import sqrt

        position = state[0:3]
        velocity = state[3:6]
        mass = state[6]

        gravity = self.asteroid.gravity_at_position(state[0:3])
        gravity_acceleration = [val / mass for val in gravity]
        thrust_acceleration = [val / mass for val in thrust]

        angular_velocity = self.asteroid.angular_velocity_at_time(time)
        angular_velocity_mul2 = [2.0 * val for val in angular_velocity]
        angular_acceleration = self.asteroid.angular_acceleration_at_time(time)


        euler_acceleration = cross_product(angular_acceleration, position)
        centrifugal_acceleration = cross_product(angular_velocity, cross_product(angular_velocity, position))
        coriolis_acceleration = cross_product(angular_velocity_mul2, velocity)

        # print("G: %f" % sqrt(sum([val * val for val in gravity_acceleration])))
        # print("O: %f" % sqrt(sum([(val_pert - val_cor - val_eu - val_centr) ** 2
        # for val_pert, val_cor, val_eu, val_centr in
        # zip(perturbations_acceleration, coriolis_acceleration, euler_acceleration, centrifugal_acceleration)])))

        d_dt_state = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        for i in range(3):
            d_dt_state[i] = velocity[i]
            d_dt_state[3+i] = perturbations_acceleration[i]\
                              + gravity_acceleration[i]\
                              + thrust_acceleration[i]\
                              - coriolis_acceleration[i]\
                              - euler_acceleration[i]\
                              - centrifugal_acceleration[i]

        # derivative of mass
        d_dt_state[6] = sqrt(thrust[0] ** 2 + thrust[1] ** 2 + thrust[2] ** 2) \
                        / self._earth_acceleration_mul_spacecraft_specific_impulse

        return d_dt_state

    # External perturbations acceleration
    def simulate_perturbations(self):
        from numpy import random

        mean = 0.0
        variance = 1e-7
        spacecraft_mass = self.spacecraft_state[6]
        return [spacecraft_mass * random.normal(mean, variance),
                spacecraft_mass * random.normal(mean, variance),
                spacecraft_mass * random.normal(mean, variance)]

