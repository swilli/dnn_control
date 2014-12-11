from boost_simulator import BoostSimulator
from numpy import random
from time import time
from utility import sample_point_outside_ellipsoid, cross_product

signs = [-1.0, 1.0]

# Simulation settings
time_to_run = 24.0 * 60.0 * 60.0  # [s]

# Asteroid settings
semi_axis = [random.uniform(8000.0, 12000.0), random.uniform(4000.0, 7500.0), random.uniform(1000.0, 3500.0)]  # [m]
density = random.uniform(1500.0, 3000.0)  # [kg/m^3]
angular_velocity = [random.choice(signs) * random.uniform(0.0002, 0.0008),
                    0.0,
                    random.choice(signs) * random.uniform(0.0002, 0.0008)]  # [1/s]

time_bias = 0.0  # random.uniform(0.0, 60.0 * 60.0 * 6.0)  # [s]

# Spacecraft settings
spacecraft_position = sample_point_outside_ellipsoid(semi_axis, 4.0)
spacecraft_velocity = [-val for val in cross_product(angular_velocity, spacecraft_position)]  # [m/s]
spacecraft_mass = 1000.0  # [kg]
spacecraft_specific_impulse = 200.0  # [s]

# Controller settings
control_frequency = 10.0  # [Hz]
target_position = [random.uniform(-500.0, 500.0) + pos for pos in spacecraft_position]
sensor_noise = 0.05
perturbation_noise = 1e-7

simulator = BoostSimulator(semi_axis, density, angular_velocity, time_bias,
                            spacecraft_position, spacecraft_velocity, spacecraft_mass, spacecraft_specific_impulse,
                            target_position, control_frequency, sensor_noise, perturbation_noise)

print("running simulation ...")
start = time()
result = simulator.run(time_to_run, False)
end = time()
print("{0} seconds simulation time took {1} real time to compute (x{2}).".format(result[0], end - start,
                                                                             result[0] / (end - start)))
positions = result[1]
heights = result[2]

from os import remove

path_to_file = "../../../../results/states.txt"

try:
    remove(path_to_file)
except OSError:
    pass

log_file = open(path_to_file, "a")
log_file.write("{0},\t{1},\t{2},\t{3}\n".format(semi_axis[0], semi_axis[1], semi_axis[2], control_frequency))
for pos, height in zip(positions,heights):
    log_file.write("{0},\t{1},\t{2},\t{3},\t{4},\t{5}\n".format(pos[0], pos[1], pos[2],
                                                    height[0], height[1], height[2]))
log_file.close()
