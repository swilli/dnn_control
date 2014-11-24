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

        angular_velocity = self.asteroid.angular_velocity_at_time(time)
        angular_velocity_mul2 = [2.0 * val for val in angular_velocity]
        angular_acceleration = self.asteroid.angular_acceleration_at_time(time)

        centrifugal_acceleration = cross_product(angular_velocity, cross_product(angular_velocity, position))
        coriolis_acceleration = cross_product(angular_velocity_mul2, velocity)
        euler_acceleration = cross_product(angular_acceleration, position)

        distance, surface_point = self.asteroid.distance_to_surface_at_position(position)

        height = [val_pos - val_surf for val_pos, val_surf in zip(position, surface_point)]

        height_norm_pow2 = distance * distance
        norm_height = distance

        velocity_dot_height = sum([val_height * val_velocity for val_height, val_velocity in zip(velocity, height)])
        scaling = velocity_dot_height / height_norm_pow2

        velocity_vertical = [scaling * val_height for val_height in height]
        velocity_remaining = [vel_total - vel_vert for vel_total, vel_vert in zip(velocity, velocity_vertical)]

        norm_vel_vertical = sqrt(sum([val * val for val in velocity_vertical]))
        norm_vel_remaining = sqrt(sum([val * val for val in velocity_remaining]))

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