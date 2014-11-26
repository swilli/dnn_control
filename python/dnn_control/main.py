from numpy import random

from pidcontroller import PIDController
from sensorsimulator import SensorSimulator
from simulator import Simulator
from asteroid import Asteroid
from time import time

# Video Settings
VIDEO_SPEED = 100.0

# Simulation settings
TIME = 24.0 * 60.0 * 60.0  # [s]
TARGET_POSITION = [1000.0, 1000.0, 1000.0]  # [m]

# Asteroid settings
signs = [-1.0, 1.0]
AXIS = [random.uniform(1000.0, 2000.0), random.uniform(2500.0, 3500.0), random.uniform(4000.0, 5000.0)]  # [m]
AXIS_C, AXIS_B, AXIS_A = AXIS

DENSITY = 2000.0  # random.uniform(2500.0, 3500.0)  # [kg/m^3]
ANGULAR_VELOCITY = [random.choice(signs) * random.uniform(0.0002, 0.0004),
                    0.0,
                    random.choice(signs) * random.uniform(0.0002, 0.0004)]  # [1/s]
TIME_BIAS = random.uniform(0.0, 60.0 * 60.0 * 6.0)  # [s]

# Spacecraft settings
POSITION = [random.uniform(1000.0, 2000.0), random.uniform(2500.0, 3500.0), random.uniform(4000.0, 5000.0)]  # [m]
POSITION = [val * 1.1 * random.choice(signs) for val in POSITION]
VELOCITY = [omega * pos for omega, pos in zip(ANGULAR_VELOCITY, POSITION)]  # [m/s]
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
positions, velocities, heights, velocities_vertical, velocities_remaining = simulator.run(TIME, True)
end = time()

duration = end - start

print("Simulation done. {0} seconds took {1} seconds to simulate (x{2}).".format(TIME, duration, TIME / duration))
# Visualize trajectory
'''try:
    input("Press enter to continue...")
except SyntaxError:
    pass

visualize(asteroid, positions, velocities, heights, velocities_vertical, velocities_remaining, VIDEO_SPEED * CONTROL_FREQUENCY)

print("done.")
'''