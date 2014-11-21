import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
from numpy import random

from pidcontroller import PIDController
from sensorsimulator import SensorSimulator
from simulator import Simulator
from asteroid import Asteroid
from constants import PI


# Simulation settings
TIME = 100.0  # [s]
TARGET_POSITION = [1000.0, 1000.0, 1000.0]  # [m]

# Asteroid settings
AXIS = []
[AXIS.append(random.uniform(1000.0, 5000.0)) for i in range(3)]
AXIS_C, AXIS_B, AXIS_A = sorted(AXIS)  # [m]

DENSITY = random.uniform(1500.0, 2500.0)  # [kg/m^3]
ANGULAR_VELOCITY = [random.uniform(-0.02 * PI, 0.02 * PI), 0.0, random.uniform(-0.02 * PI, 0.02 * PI)]  # [1/s]
TIME_BIAS = random.uniform(0.0, 60.0 * 60.0 * 6.0)  # [s]

# Spacecraft settings
POSITION = [6000, 6000, 6000]  # [m]
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
'''
fig = pyplot.figure()
ax = fig.gca(projection="3d")
ax.plot(positions[:, 0], positions[:, 1], positions[
        :, 2], label="spacecraft trajectory")
pyplot.plot([positions[0][0]], [positions[0][1]],
            [positions[0][2]], 'rD', label="start")
pyplot.plot([positions[-1][0]], [positions[-1][1]],
            [positions[-1][2]], 'bD', label="end")
ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
pyplot.show()
'''