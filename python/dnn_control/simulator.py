class Simulator:

    '''
        This class implements the interaction between stepwise integration of the ode system,
        artificial sensor data generation, control polling, and result logging.
    '''

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

    # Simulates random noise in the dynamical system (which is fed into the ODE system "system_")
    def _simulate_perturbations(self):
        from numpy import random

        mass = self._integrator.y[6]
        return [mass * random.normal(0.0, self._perturbation_noise),
                mass * random.normal(0.0, self._perturbation_noise),
                mass * random.normal(0.0, self._perturbation_noise)]

    # Before running the simulator, it has to be initialized with a spacecraft configuration
    def init_spacecraft(self, position, velocity, mass, specific_impulse):
        from constants import EARTH_ACCELERATION

        # Transform position, velocity and mass to state
        self._integrator.set_initial_value(position + velocity + [mass], 0.0)

        # Cache g * Isp
        self._system.coef_earth_acceleration_mul_specific_impulse = 1.0 / (specific_impulse * EARTH_ACCELERATION)

    # Initialize spacecraft only by specifying the spacecraft's specific impulse (useful for the function next_state)
    def init_spacecraft_specific_impulse(self, specific_impulse):
        from constants import EARTH_ACCELERATION

        # Cache g * Isp
        self._system.coef_earth_acceleration_mul_specific_impulse = 1.0 / (specific_impulse * EARTH_ACCELERATION)

    # Implements F: S x A x T -> S : F(s,a,t) = s' (Useful for RL?)
    def next_state(self, state, thrust, time):
        # Assign state
        self._integrator.set_initial_value(state, time)
        self._system.thrust = thrust

        # Simulate perturbations, directly write it to the ode system
        self._system.perturbations_acceleration = self._simulate_perturbations()

        # Integrate for one time step _control_interval
        self._integrator.integrate(time + self._control_interval)

        # Extract new state out of system
        return self._integrator.y[:]

    # Simulates the system for time "time". Logs the states if "log_data" is enabled,
    # Returns the number of iterations the simulator made to get to the specified time.
    def run(self, time, log_data=False):
        from numpy import zeros

        dt = self._control_interval
        iterations = int(time / dt)

        if log_data:
            self._log_states = zeros([iterations, 2, 3])

        current_time = 0.0
        # Stepwise integration for "iterations" iterations
        try:
            for iteration in xrange(iterations):
                if log_data:
                    # Log position and height, if enabled
                    position = list(self._integrator.y[0:3])
                    surface_position = self._system.asteroid.nearest_point_on_surface_to_position(position)[0]
                    for i in [0, 1, 2]:
                        self._log_states[iteration][0][i] = position[i]
                        self._log_states[iteration][1][i] = position[i] - surface_position[i]

                # Simulate perturbations, directly write it to the ode system
                self._system.perturbations_acceleration = self._simulate_perturbations()

                # Simulate sensor data
                sensor_data = self._sensor_simulator.simulate(self._integrator.y,
                                                             self._system.perturbations_acceleration,
                                                             current_time)

                # Poll controller for control thrust, write it to the ode system
                self._system.thrust = self._spacecraft_controller.get_thrust_for_sensor_data(sensor_data)

                # Integrate the system for _control_interval time
                self._integrator.integrate(current_time + dt)
                if not self._integrator.successful():
                    print("Something BAD happened...")
                    exit(1)

                current_time += dt

                # Check if spacecraft is out of fuel
                if self._integrator.y[6] <= 0.0:
                    print("The spacecraft is out of fuel.")
                    break

        except RuntimeError as error:
            print("The spacecraft crashed into the asteroid's surface.")

        return current_time

    # Writes the logged data from Run to the file given by "path_to_file"
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
