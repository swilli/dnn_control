class SensorSimulator:

    def __init__(self, asteroid):
        self.asteroid = asteroid

    def simulate(self, state, perturbations_acceleration, time):
        from utility import cross_product
        from math import sqrt
        from numpy import random

        sensor_data = [0.0, 0.0, 0.0, 0.0, 0.0]

        position = state[0:3]
        velocity = state[3:6]
        mass = state[6]

        gravity = self.asteroid.gravity_at_position(state[0:3])
        gravity_acceleration = [val / mass for val in gravity]

        angular_velocity, angular_acceleration = self.asteroid.angular_velocity_and_acceleration_at_time(time)
        angular_velocity_mul2 = [2.0 * val for val in angular_velocity]

        centrifugal_acceleration = cross_product(angular_velocity, cross_product(angular_velocity, position))
        coriolis_acceleration = cross_product(angular_velocity_mul2, velocity)
        euler_acceleration = cross_product(angular_acceleration, position)

        distance, surface_point = self.asteroid.distance_to_surface_at_position(position)

        height = [position[0] - surface_point[0],
                  position[1] - surface_point[1],
                  position[2] - surface_point[2]]

        height_norm_pow2 = distance * distance
        norm_height = distance

        velocity_dot_height = velocity[0] * height[0] + velocity[1] * height[1] + velocity[2] * height[2]
        scaling = velocity_dot_height / height_norm_pow2

        velocity_vertical = [scaling * val_height for val_height in height]
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
        for i in range(3):
            sensor_data[2 + i] = perturbations_acceleration[i] \
                                 + gravity_acceleration[i] \
                                 - coriolis_acceleration[i]\
                                 - euler_acceleration[i]\
                                 - centrifugal_acceleration[i]

        variances = [0.05] * 5
        for i in range(5):
            sensor_data[i] += sensor_data[i] * random.normal(0, variances[i])
        return sensor_data, height, velocity_vertical, velocity_remaining