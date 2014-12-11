class SensorDataGenerator:

    def __init__(self, path_to_folder, control_frequency, data_set_time):
        self._data_set_time = data_set_time
        self._control_frequency = control_frequency
        self._path_to_folder = path_to_folder


    def generate(self, num_datasets, prefix=""):
        from numpy import random
        from time import gmtime, strftime
        from  sys import stdout
        from boost_simulator.utility import sample_sign, sample_point_outside_ellipsoid, cross_product
        from filewriter import FileWriter
        from boost_simulator import boost_simulator
        Simulator = boost_simulator.BoostSimulator

        for data_iter in xrange(num_datasets):
            time_to_run = self._data_set_time
            semi_axis = [random.uniform(8000.0, 12000.0), random.uniform(4000.0, 7500.0), random.uniform(1000.0, 3500.0)]
            density = random.uniform(1500.0, 3000.0)
            angular_velocity = [sample_sign() * random.uniform(0.0002, 0.0008),
                                0.0,
                                sample_sign() * random.uniform(0.0002, 0.0008)]
            time_bias = 0.0

            spacecraft_position = sample_point_outside_ellipsoid(semi_axis, 4.0)
            spacecraft_velocity = [-val for val in cross_product(angular_velocity, spacecraft_position)]
            spacecraft_specific_impulse = 200.0
            spacecraft_mass = random.uniform(500.0, 1500.0)

            control_frequency = self._control_frequency

            target_position = [random.uniform(-500.0, 500.0) + pos for pos in spacecraft_position]

            sensor_noise = 0.05
            perturbation_noise = 1e-7

            simulator = Simulator(semi_axis, density, angular_velocity, time_bias,
                      spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse,
                      target_position, control_frequency, sensor_noise, perturbation_noise)

            result = simulator.run(time_to_run, True)
            sensor_data = result[3]

            path_to_file = self._path_to_folder

            if prefix == "":
                path_to_file += strftime("%d_%m_%H_%I_%M_%S_", gmtime())
            else:
                path_to_file += prefix + "_"

            path_to_file += "gen" + str(data_iter + 1) + ".txt"

            stdout.write("{0}: writing sensor data to file {1} ... ".format(data_iter + 1, path_to_file))
            writer = FileWriter()
            writer.create_sensor_data_file(path_to_file, control_frequency, time_to_run,
                                           semi_axis, density, angular_velocity, time_bias,
                                           spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse,
                                           target_position, sensor_data)
            stdout.write("done.\n")