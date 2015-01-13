class FullStateSensorSimulator:

    def __init__(self, asteroid, sensor_noise):
        self._asteroid = asteroid
        self._sensor_noise = sensor_noise

    # Generates (simulates) sensor data based on the current spacecraft state "state" and time "time"
    def simulate(self, state, perturbations_acceleration, time):
        return state