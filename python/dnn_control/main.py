from numpy import random
from pidcontroller import PIDController
from sensorsimulator import SensorSimulator
from simulator import Simulator
from asteroid import Asteroid
from time import time
from visualization import visualize, visualize_full
from constants import PI
from math import cos, sin

# Video Settings
VIDEO_SPEED = 100  # VIDEO_SPEED times real time
ALL_DATA = True

# Simulation settings
TIME = 24.0 * 60.0 * 60.0  # [s]
TARGET_POSITION = [1000.0, 1000.0, 1000.0]  # [m]

# Asteroid settings
signs = [-1.0, 1.0]
c_axis_n = 1.0
b_axis_n = random.uniform(1.1, 2.0)
a_axis_n = random.uniform(1.1 * b_axis_n, 4.0)
distance = random.uniform(100.0, 8000.0)
AXIS = [a_axis_n * distance,
        b_axis_n * distance,
        c_axis_n * distance]  # [m]
AXIS_A, AXIS_B, AXIS_C = AXIS

DENSITY = random.uniform(1500.0, 3000.0)  # [kg/m^3]
ANGULAR_VELOCITY = [random.choice(signs) * random.uniform(0.0002, 0.0004),
                    0.0,
                    random.choice(signs) * random.uniform(0.0002, 0.0004)]  # [1/s]
ANGULAR_VELOCITY = [val * 1e-2 for val in ANGULAR_VELOCITY]
print(ANGULAR_VELOCITY)

TIME_BIAS = random.uniform(0.0, 60.0 * 60.0 * 6.0)  # [s]

# Spacecraft settings
u = random.uniform(0.0, 2.0 * PI)
v = random.uniform(0.0, PI)
POSITION = [(1.1 + random.rand() * 3.0) * AXIS_A * cos(u) * sin(v),
            (1.1 + random.rand() * 3.0) * AXIS_B * sin(u) * sin(v),
            (1.1 + random.rand() * 3.0) * AXIS_C * cos(v)]  # [m]
POSITION = [val * random.choice(signs) for val in POSITION]
VELOCITY = [omega * pos for omega, pos in zip(ANGULAR_VELOCITY, POSITION)]  # [m/s]
VELOCITY = [0.0] * 3
MASS = 1000.0  # [kg]
SPECIFIC_IMPULSE = 200.0  # [s]

# Controller settings
CONTROL_FREQUENCY = 10.0  # [Hz]

# Instantiate asteroid
asteroid = Asteroid(AXIS_A, AXIS_B, AXIS_C, DENSITY, ANGULAR_VELOCITY, TIME_BIAS)

# Instantiate sensor simulator
sensor_simulator = SensorSimulator(asteroid)
# Instantiate controller
controller = PIDController(TARGET_POSITION, 1.0 / CONTROL_FREQUENCY)
# Instantiate simulator
simulator = Simulator(asteroid, POSITION, VELOCITY, MASS, SPECIFIC_IMPULSE,
                      sensor_simulator, controller, CONTROL_FREQUENCY)

# Run simulator
start = time()
if ALL_DATA:
    positions, velocities, heights, velocities_vertical, velocities_remaining, accelerations_perturbations, \
    accelerations_centrifugal, accelerations_coriolis,\
    accelerations_euler, accelerations_gravity, angular_velocities, angular_accelerations = simulator.run(TIME, True)
else:
    positions = simulator.run(TIME, False)
end = time()

duration = end - start

print("ALL_DATA={0} simulation done. {1} seconds took {2} seconds to simulate (x{3}).".format(ALL_DATA,
                                                                                              TIME, duration, TIME / duration))

# Visualize trajectory
'''try:
    input("Press enter to continue...")
except SyntaxError:
    pass
'''

if ALL_DATA:
    visualize_full(asteroid, positions, velocities, heights, velocities_vertical, velocities_remaining,
          accelerations_perturbations, accelerations_centrifugal, accelerations_coriolis, accelerations_euler,
          accelerations_gravity, angular_velocities, angular_accelerations, VIDEO_SPEED * CONTROL_FREQUENCY)
else:
    visualize(asteroid, positions, VIDEO_SPEED * CONTROL_FREQUENCY)

print("done.")
