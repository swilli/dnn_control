import matplotlib
import numpy 
import matplotlib.pyplot as pyplot
from mpl_toolkits.mplot3d import Axes3D
import constants
import pidcontroller
import sensorsimulator
import simulator



# Simulation settings
TIME = 120 # [s]

# Asteroid settings
A_SEMI_AXIS = 4000 # [m]
B_SEMI_AXIS = 2000 # [m]
C_SEMI_AXIS = 1000 # [m]
DENSITY = 2000 # [kg/m^3]
THETA = 0#constants.PI/2
OMEGA_ZERO = 0.1

# Spacecraft settings
POSITION = [1e4, 1e4, 1e4] # [m]
VELOCITY = [0, 0, 0] # [m/s]
MASS = 1000 # [kg]
SPECIFIC_IMPULSE = 200

# Controller settings
CONTROL_FREQUENCY = 10 # [Hz]

# Instantiate sensor simulator
sensor_simulator = sensorsimulator.SensorSimulator()
# Instantiate controller
controller = pidcontroller.PIDController()
# Instantiate simulator
simulator = simulator.Simulator(A_SEMI_AXIS, B_SEMI_AXIS, C_SEMI_AXIS, DENSITY, THETA, OMEGA_ZERO, POSITION, VELOCITY, MASS, SPECIFIC_IMPULSE, sensor_simulator, controller, CONTROL_FREQUENCY)

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
