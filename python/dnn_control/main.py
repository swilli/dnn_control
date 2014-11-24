from numpy import random

from pidcontroller import PIDController
from sensorsimulator import SensorSimulator
from simulator import Simulator
from asteroid import Asteroid
from constants import PI
from visualization import visualize, plot_3d

signs = [-1.0, 1.0]

# Simulation settings
TIME = 1000.0  # [s]
TARGET_POSITION = [1000.0, 1000.0, 1000.0]  # [m]

# Asteroid settings
AXIS = [random.uniform(1000.0, 2000.0), random.uniform(2500.0, 3500.0), random.uniform(4000.0, 5000.0)]  # [m]
AXIS_C, AXIS_B, AXIS_A = sorted(AXIS)

DENSITY = random.uniform(1500.0, 2500.0)  # [kg/m^3]
ANGULAR_VELOCITY = [random.uniform(-0.02 * PI, 0.02 * PI), 0.0, random.uniform(-0.02 * PI, 0.02 * PI)]  # [1/s]
TIME_BIAS = random.uniform(0.0, 60.0 * 60.0 * 6.0)  # [s]

# Spacecraft settings
POSITION = [random.uniform(1000.0, 2000.0), random.uniform(2500.0, 3500.0), random.uniform(4000.0, 5000.0)]  # [m]
POSITION = [val * 1.1 * random.choice(signs) for val in POSITION]
VELOCITY = [0.0, 0.0, 0.0]  # [m/s]
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
positions = simulator.run(TIME, True)

# Visualize trajectory
visualize(asteroid, positions, CONTROL_FREQUENCY)
