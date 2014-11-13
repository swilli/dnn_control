import matplotlib
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
import constants
from pidcontroller import *
from sensorsimulator import *
from simulator import *
from asteroid import *


# Simulation settings
TIME = 100 # [s]
TARGET_POSITION = [10000.0, 10000.0, 10000.0] # [m]

# Asteroid settings
INERTIA_X = 4567.0 # [m]
INERTIA_Y = 2345.0 # [m]
INERTIA_Z = 1234.0 # [m]
DENSITY = 2000.0 # [kg/m^3]
ANGULAR_VELOCITY = [0.00029565148,0.00029565148,0.00029565148] # [1/s]
TIME_BIAS = 0.0 # [s]

# Spacecraft settings
POSITION = [10000.0, 10000.0, 10000.0] # [m]
VELOCITY = [0.0, 0.0, 0.0] # [m/s]
MASS = 1000.0 # [kg]
SPECIFIC_IMPULSE = 200.0 # [s]

# Controller settings
CONTROL_FREQUENCY = 10.0 # [Hz]

# Instantiate asteroid
asteroid = Asteroid(INERTIA_X, INERTIA_Y, INERTIA_Z, DENSITY, ANGULAR_VELOCITY, TIME_BIAS)

# Instantiate sensor simulator
sensor_simulator = SensorSimulator()
# Instantiate controller
controller = PIDController(TARGET_POSITION, 1.0/CONTROL_FREQUENCY)
# Instantiate simulator
simulator = Simulator(asteroid, POSITION, VELOCITY, MASS, SPECIFIC_IMPULSE, sensor_simulator, controller, CONTROL_FREQUENCY)

# Run simulator
positions = simulator.run(TIME, True)

# Visualize trajectory
fig = pyplot.figure()
ax = fig.gca(projection="3d")
ax.plot(positions[:,0],positions[:,1],positions[:,2], label="spacecraft trajectory")
pyplot.plot([positions[0][0]],[positions[0][1]],[positions[0][2]], 'rD', label="start")
pyplot.plot([positions[-1][0]],[positions[-1][1]],[positions[-1][2]], 'bD', label="end")
ax.legend()
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')
pyplot.show()
