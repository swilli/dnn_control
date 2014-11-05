import constants
import pidcontroller
import sensorsimulator
import simulator

# Simulation settings
TIME = 100 # [s]

# Asteroid settings
C_SEMI_AXIS = 4000 # [m]
B_SEMI_AXIS = 1000 # [m]
A_SEMI_AXIS = 500 # [m]
DENSITY = 2000 # [kg/m^3]
THETA = constants.PI/2
OMEGA_ZERO = 0.1

# Spacecraft settings
POSITION = [1,1,1] # [m]
VELOCITY = [0,0,0] # [m/s]
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
simulator.run(TIME)