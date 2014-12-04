from numpy import random
from pidcontroller import PIDController
from sensorsimulator import SensorSimulator
from simulator import Simulator
from asteroid import Asteroid
from time import time
from visualization import visualize, visualize_full
from constants import PI
from math import cos, sin
from utility import cross_product, sample_position_outside_ellipsoid

path_to_file = "../../../results/states.txt"

signs = [-1.0, 1.0]

# Video Settings
video_speed = 100  # VIDEO_SPEED times real time
all_data = True

# Simulation settings
time_to_run = 24.0 * 60.0 * 60.0  # [s]
target_position = [1000.0, 1000.0, 1000.0]  # [m]

# Asteroid settings
c_axis_n = 1.0
b_axis_n = random.uniform(1.1, 2.0)
a_axis_n = random.uniform(1.1 * b_axis_n, 4.0)
distance = random.uniform(100.0, 8000.0)
semi_axis = [a_axis_n * distance, b_axis_n * distance, c_axis_n * distance]  # [m]
density = random.uniform(1500.0, 3000.0)  # [kg/m^3]
angular_velocity = [random.choice(signs) * random.uniform(0.0002, 0.0008),
                    0.0,
                    random.choice(signs) * random.uniform(0.0002, 0.0008)]  # [1/s]

time_bias = 0.0  # random.uniform(0.0, 60.0 * 60.0 * 6.0)  # [s]

# Spacecraft settings
spacecraft_position = sample_position_outside_ellipsoid(semi_axis, 4.0)
spacecraft_velocity = [-val for val in cross_product(angular_velocity, spacecraft_position)]  # [m/s]
spacecraft_mass = 1000.0  # [kg]
spacecraft_specific_impulse = 200.0  # [s]

# Controller settings
control_frequency = 10.0  # [Hz]

sensor_noise = 0.05
perturbation_noise = 1e-7

# Instantiate asteroid
asteroid = Asteroid(semi_axis, density, angular_velocity, time_bias)

# Instantiate sensor simulator
sensor_simulator = SensorSimulator(asteroid, sensor_noise)
# Instantiate controller
controller = PIDController(target_position, 1.0 / control_frequency)
# Instantiate simulator
simulator = Simulator(asteroid, sensor_simulator, controller, control_frequency, perturbation_noise)

simulator.init_spacecraft(spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse)

print("running simulation ...")
start = time()
simulator.run(time_to_run, True)
end = time()
print("{0} seconds simulation time took {1} real time to compute (x{2}).".format(time_to_run, end - start, time_to_run / (end - start)))
print("writing results to file ...")
simulator.flush_log_to_file(path_to_file)
print("done.")
