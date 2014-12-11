class ODESystem:
    '''
    This class represents the dynamics in a rotating reference frame (which we have around an asteroid) using ordinary
    differential equations. Implementation is inspired from "Control of Hovering Spacecraft Using Altimetry" by
    S. Sawai et. al.
    '''

    def __init__(self, asteroid):
        self.asteroid = asteroid
        self.thrust = [0.0, 0.0, 0.0]
        self.coef_earth_acceleration_mul_specific_impulse = 0.0
        self.perturbations_acceleration = [0.0, 0.0, 0.0]

    # This function gets called by the scipy ode stepper to integrate the system.
    # The function computes d/dt x = f(x, t). Or in our case: "d_state_dt" = this(state, time)
    def dynamics(self, time, state):
        from math import sqrt

        # implements eq(1) of "Control of Hovering Spacecraft Using Altimetry" by S. Sawai et. al.
        # r'' + 2w x r' + w' x r + w x (w x r) = Fc + Fg,
        # whereas Fc = control acceleration and Fg = gravity acceleration

        d_state_dt = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        position = [state[0], state[1], state[2]]
        velocity = [state[3], state[4], state[5]]
        mass = state[6]

        # 1/m
        coef_mass = 1.0 / mass

        # g
        gravity_acceleration = self.asteroid.gravity_at_position(position)

        # w, w'
        angular_velocity, angular_acceleration = self.asteroid.angular_velocity_and_acceleration_at_time(time)

        # Fg, Fc
        thrust_acceleration = [0.0, 0.0, 0.0]
        for i in [0, 1, 2]:
            gravity_acceleration[i] *= coef_mass
            thrust_acceleration[i] = self.thrust[i] * coef_mass

        d_state_dt[0] = state[3]
        d_state_dt[1] = state[4]
        d_state_dt[2] = state[5]

        d_state_dt[3] = self.perturbations_acceleration[0] + gravity_acceleration[0] + thrust_acceleration[0] \
                        - 2.0 * angular_velocity[1] * velocity[2] + 2.0 * angular_velocity[2] * velocity[1] \
                        - angular_acceleration[1] * position[2] + angular_acceleration[2] * position[1]\
                        - angular_velocity[0] * angular_velocity[1] * position[1] \
                        + angular_velocity[1] * angular_velocity[1] * position[0] \
                        + angular_velocity[2] * angular_velocity[2] * position[0]\
                        - angular_velocity[0] * angular_velocity[2] * position[2]

        d_state_dt[4] = self.perturbations_acceleration[1] + gravity_acceleration[1] + thrust_acceleration[1] \
                        - 2.0 * angular_velocity[2] * velocity[0] + 2.0 * angular_velocity[0] * velocity[2] \
                        - angular_acceleration[2] * position[0] + angular_acceleration[0] * position[2] \
                        - angular_velocity[1] * angular_velocity[2] * position[2] \
                        + angular_velocity[2] * angular_velocity[2] * position[1] \
                        + angular_velocity[0] * angular_velocity[0] * position[1]\
                        - angular_velocity[0] * angular_velocity[1] * position[0]

        d_state_dt[5] = self.perturbations_acceleration[2] + gravity_acceleration[2] + thrust_acceleration[2] \
                        - 2.0 * angular_velocity[0] * velocity[1] + 2.0 * angular_velocity[1] * velocity[0] \
                        - angular_acceleration[0] * position[1] + angular_acceleration[1] * position[0] \
                        - angular_velocity[0] * angular_velocity[2] * position[0] \
                        + angular_velocity[0] * angular_velocity[0] * position[2] \
                        + angular_velocity[1] * angular_velocity[1] * position[2] \
                        - angular_velocity[1] * angular_velocity[2] * position[1]

        d_state_dt[6] = -sqrt(self.thrust[0] * self.thrust[0]
                             + self.thrust[1] * self.thrust[1]
                             + self.thrust[2] * self.thrust[2]) * self.coef_earth_acceleration_mul_specific_impulse

        return d_state_dt
