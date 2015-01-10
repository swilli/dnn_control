class FileWriter:

    def __init__(self):
        pass

    def create_visualization_file(self, path_to_file, control_frequency, semi_axis, positions, heights):
        from os import remove

        try:
            remove(path_to_file)
        except OSError:
            pass

        result_file = open(path_to_file, "a")
        result_file.write("{0},\t{1},\t{2},\t{3}\n".format(semi_axis[0], semi_axis[1], semi_axis[2], control_frequency))
        for pos, height in zip(positions, heights):
            result_file.write("{0},\t{1},\t{2},\t{3},\t{4},\t{5}\n".format(pos[0], pos[1], pos[2],
                                                                    height[0], height[1],height[2]))
        result_file.close()


    def create_sensor_data_file(self, path_to_file, control_frequency, time_to_run,
                                semi_axis, density, angular_velocity, time_bias,
                                spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse,
                                target_position, sensor_data):
        from os import remove

        try:
            remove(path_to_file)
        except OSError:
            pass

        result_file = open(path_to_file, "a")
        result_file.write("# target position: {0} m\n".format(target_position))
        result_file.write("#\n")
        result_file.write("# control frequency: {0} Hz\n".format(control_frequency))
        result_file.write("# simulation time: {0} s\n".format(time_to_run))
        result_file.write("#\n")
        result_file.write("# asteroid:\n")
        result_file.write("#  density: {0} kg/m^3\n".format(density))
        result_file.write("#  time bias: {0} s\n".format(time_bias))
        result_file.write("#  semi axis: {0} m\n".format(semi_axis))
        result_file.write("#  angular velocity: {0}\n".format(angular_velocity))
        result_file.write("#\n")
        result_file.write("# spacecraft:\n")
        result_file.write("#  mass: {0} kg\n".format(spacecraft_mass))
        result_file.write("#  specific impulse: {0} s\n".format(spacecraft_specific_impulse))
        result_file.write("#  position: {0} m\n".format(spacecraft_position))
        result_file.write("#  velocity: {0} m/s\n".format(spacecraft_velocity))
        result_file.write("#\n")
        for data in sensor_data:
            result_file.write("{0}".format(data[0]))
            for i in range(1, len(data)):
                result_file.write(", {0}".format(data[i]))

            result_file.write("\n")

        result_file.close()