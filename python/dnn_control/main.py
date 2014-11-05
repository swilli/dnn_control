import pidcontroller
import simulator

# Controller settings
CONTROL_FREQUENCY = 10 # [Hz]

# Spacecraft settings
POSITION = [1,1,1] # [m]
VELOCITY = [0,0,0] # [m/s]
MASS = 1000 # [kg]

# Simulation settings
TIME = 100 # [s]


# Select controller
controller = pidcontroller.PIDController()

# Instantiate simulator
simulator = simulator.Simulator(controller,CONTROL_FREQUENCY,POSITION,VELOCITY,MASS)

# Run simulator
simulator.run(TIME)