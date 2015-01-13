class SensorSimulator5D:

    '''
    This class generates the artificial the sensor data required for a controller.
    '''

    def __init__(self, asteroid, sensor_noise):
        self._asteroid = asteroid
        self._sensor_noise = sensor_noise

    # Generates (simulates) sensor data based on the current spacecraft state "state" and time "time"
    def simulate(self, state, perturbations_acceleration, time):
        from math import sqrt
        from numpy import random

        sensor_data = [0.0, 0.0, 0.0, 0.0, 0.0]

        position = [state[0], state[1], state[2]]
        velocity = [state[3], state[4], state[5]]
        mass = state[6]
        coef_mass = 1.0 / mass

        gravity_acceleration = self._asteroid.gravity_at_position(position)
        for i in [0, 1, 2]:
            gravity_acceleration[i] *= coef_mass

        angular_velocity, angular_acceleration = self._asteroid.angular_velocity_and_acceleration_at_time(time)

        surface_point, distance = self._asteroid.nearest_point_on_surface_to_position(position)

        height = [position[0] - surface_point[0],
                  position[1] - surface_point[1],
                  position[2] - surface_point[2]]

        height_norm_pow2 = distance * distance
        norm_height = distance

        velocity_dot_height = velocity[0] * height[0] + velocity[1] * height[1] + velocity[2] * height[2]
        scaling = velocity_dot_height / height_norm_pow2

        velocity_vertical = [scaling * height[0], scaling * height[1], scaling * height[2]]
        velocity_remaining = [velocity[0] - velocity_vertical[0],
                              velocity[1] - velocity_vertical[1],
                              velocity[2] - velocity_vertical[2]]

        norm_vel_vertical = sqrt(velocity_vertical[0] * velocity_vertical[0]
                                 + velocity_vertical[1] * velocity_vertical[1]
                                 + velocity_vertical[2] * velocity_vertical[2])
        norm_vel_remaining = sqrt(velocity_remaining[0] * velocity_remaining[0]
                                  + velocity_remaining[1] * velocity_remaining[1]
                                  + velocity_remaining[2] * velocity_remaining[2])

        sensor_data[0] = norm_vel_vertical / norm_height
        sensor_data[1] = norm_vel_remaining / norm_height

        sensor_data[2] = perturbations_acceleration[0] + gravity_acceleration[0] \
                        - 2.0 * angular_velocity[1] * velocity[2] + 2.0 * angular_velocity[2] * velocity[1] \
                        - angular_acceleration[1] * position[2] + angular_acceleration[2] * position[1]\
                        - angular_velocity[0] * angular_velocity[1] * position[1] \
                        + angular_velocity[1] * angular_velocity[1] * position[0] \
                        + angular_velocity[2] * angular_velocity[2] * position[0]\
                        - angular_velocity[0] * angular_velocity[2] * position[2]

        sensor_data[3] = perturbations_acceleration[1] + gravity_acceleration[1] \
                        - 2.0 * angular_velocity[2] * velocity[0] + 2.0 * angular_velocity[0] * velocity[2] \
                        - angular_acceleration[2] * position[0] + angular_acceleration[0] * position[2] \
                        - angular_velocity[1] * angular_velocity[2] * position[2] \
                        + angular_velocity[2] * angular_velocity[2] * position[1] \
                        + angular_velocity[0] * angular_velocity[0] * position[1]\
                        - angular_velocity[0] * angular_velocity[1] * position[0]

        sensor_data[4] = perturbations_acceleration[2] + gravity_acceleration[2] \
                        - 2.0 * angular_velocity[0] * velocity[1] + 2.0 * angular_velocity[1] * velocity[0] \
                        - angular_acceleration[0] * position[1] + angular_acceleration[1] * position[0] \
                        - angular_velocity[0] * angular_velocity[2] * position[0] \
                        + angular_velocity[0] * angular_velocity[0] * position[2] \
                        + angular_velocity[1] * angular_velocity[1] * position[2] \
                        - angular_velocity[1] * angular_velocity[2] * position[1]

        for i in [0, 1, 2, 3, 4]:
            sensor_data[i] += sensor_data[i] * random.normal(0, self._sensor_noise)

        return sensor_data