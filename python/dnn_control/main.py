from time import time
from filewriter import FileWriter
from numpy import random
from utility import cross_product, sample_point_outside_ellipsoid, sample_sign
from sensordatagenerator import SensorDataGenerator
from boost_simulator import boost_simulator

Simulator = boost_simulator.BoostSimulator

NUM_NEW_DATA_SETS = 100

WRITE_SENSOR_DATA_TO_FILE = False
PATH_TO_SENSOR_DATA_FOLDER = "../../../data/"

WRITE_STATES_TO_FILE = True
PATH_TO_STATES_FILE = "../../../results/states.txt"

# Time settings
time_to_run = 24.0 * 60.0 * 60.0  # [s]
control_frequency = 10.0  # [Hz]

if WRITE_SENSOR_DATA_TO_FILE:
    generator = SensorDataGenerator(PATH_TO_SENSOR_DATA_FOLDER, control_frequency, time_to_run / 6.0)
    generator.generate(NUM_NEW_DATA_SETS)
    exit()

# Asteroid settings
c_axis_n = 1.0
b_axis_n = random.uniform(1.1, 2.0)
a_axis_n = random.uniform(1.1 * b_axis_n, 4.0)
distance = random.uniform(100.0, 8000.0)
semi_axis = [a_axis_n * distance, b_axis_n * distance, c_axis_n * distance]  # [m]
density = random.uniform(1500.0, 3000.0)  # [kg/m^3]
angular_velocity = [sample_sign() * random.uniform(0.0002, 0.0008),
                    0.0,
                    sample_sign() * random.uniform(0.0002, 0.0008)]  # [1/s]

time_bias = 0.0  # random.uniform(0.0, 60.0 * 60.0 * 6.0)  # [s]

# Spacecraft settings
spacecraft_position = sample_point_outside_ellipsoid(semi_axis, 4.0)
spacecraft_velocity = [-val for val in cross_product(angular_velocity, spacecraft_position)]  # [m/s]
spacecraft_mass = 1000.0  # [kg]
spacecraft_specific_impulse = 200.0  # [s]

# Controller settings
target_position = [random.uniform(-500.0, 500.0) + pos for pos in spacecraft_position]

# Noise settings
sensor_noise = 0.05
perturbation_noise = 1e-7
control_noise = 0.05

simulator = Simulator(semi_axis, density, angular_velocity, time_bias,
                      spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse,
                      target_position, control_frequency, sensor_noise, perturbation_noise, control_noise)

print("running simulation ...")
start = time()
result = simulator.run(time_to_run, False)
end = time()
print("{0} seconds simulation time took {1} real time to compute (x{2}).".format(result[0], end - start,
                                                                                 result[0] / (end - start)))

if WRITE_STATES_TO_FILE:
    print("writing states to file ... ")
    positions = result[1]
    heights = result[2]
    writer = FileWriter()
    writer.create_visualization_file(PATH_TO_STATES_FILE, control_frequency, semi_axis, positions, heights)
    print("done.")


