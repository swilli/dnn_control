class Simulator:

    def __init__(self, asteroid, sensor_simulator, spacecraft_controller, control_frequency, perturbation_noise):
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

        from odesystem import ODESystem
        from scipy.integrate import ode

        self._sensor_simulator = sensor_simulator
        self._spacecraft_controller = spacecraft_controller
        self._control_frequency = float(control_frequency)
        self._control_interval = 1.0 / self._control_frequency
        self._perturbation_noise = perturbation_noise

        self._system = ODESystem(asteroid)
        self._integrator = ode(self._system.dynamics)
        self._integrator.set_integrator("lsoda")
        self._log_states = []

    # External perturbations acceleration
    def _simulate_perturbations(self):
        from numpy import random

        mass = self._integrator.y[6]
        return [mass * random.normal(0.0, self._perturbation_noise),
                mass * random.normal(0.0, self._perturbation_noise),
                mass * random.normal(0.0, self._perturbation_noise)]

    def init_spacecraft(self, position, velocity, mass, specific_impulse):
        from constants import EARTH_ACCELERATION

        self._integrator.set_initial_value(position + velocity + [mass], 0.0)
        self._system.coef_earth_acceleration_mul_specific_impulse = specific_impulse * EARTH_ACCELERATION

    def init_spacecraft_specific_impulse(self, specific_impulse):
        from constants import EARTH_ACCELERATION

        self._system.coef_earth_acceleration_mul_specific_impulse = specific_impulse * EARTH_ACCELERATION

    def next_state(self, state, thrust, time):
        self._integrator.set_initial_value(state, time)
        self._system.thrust = thrust
        self._system.perturbations_acceleration = self._simulate_perturbations()

        self._integrator.integrate(time + self._control_interval)

        return self._integrator.y[:]


    # Perform the simulation for time seconds
    def run(self, time, log_data=False):
        from numpy import zeros

        dt = self._control_interval
        iterations = int(time / dt)

        if log_data:
            self._log_states = zeros([iterations, 2, 3])

        current_time = 0.0
        for iteration in xrange(iterations):
            if log_data:
                position = self._integrator.y[0:3]
                surface_position = self._system.asteroid.nearest_point_on_surface_to_position(position)[0]
                for i in xrange(3):
                    self._log_states[iteration][0][i] = position[i]
                    self._log_states[iteration][1][i] = position[i] - surface_position[i]

            # Get new perturbations
            self._system.perturbations_acceleration = self._simulate_perturbations()

            # Simulate sensor data for current spacecraft state
            sensor_data = self._sensor_simulator.simulate(self._integrator.y,
                                                         self._system.perturbations_acceleration,
                                                         current_time)

            # Get thrust from controller
            self._system.thrust = self._spacecraft_controller.get_thrust_for_sensor_data(sensor_data)

            self._integrator.integrate(current_time + dt)
            if not self._integrator.successful():
                print("Something BAD happened...")
                exit(1)

            current_time += dt

    def flush_log_to_file(self, path_to_file):
        from os import remove

        try:
            remove(path_to_file)
        except OSError:
            pass

        log_file = open(path_to_file, "a")
        log_file.write("{0},\t{1},\t{2},\t{3}\n".format(self._system.asteroid.get_semi_axis(0),
                                                  self._system.asteroid.get_semi_axis(1),
                                                  self._system.asteroid.get_semi_axis(2),
                                                  self._control_frequency))
        for state in self._log_states:
            log_file.write("{0},\t{1},\t{2},\t{3},\t{4},\t{5}\n".format(state[0][0], state[0][1], state[0][2],
                                                            state[1][0], state[1][1], state[1][2]))
        log_file.close()
        self._log_states = []
