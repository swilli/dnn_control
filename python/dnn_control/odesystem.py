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
        from utility import cross_product
        from math import sqrt

        # implements eq(1) of "Control of Hovering Spacecraft Using Altimetry" by S. Sawai et. al.
        # r'' + 2w x r' + w' x r + w x (w x r) = Fc + Fg,
        # whereas Fc = control acceleration and Fg = gravity acceleration

        d_state_dt = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        position = list(state[0:3])
        velocity = list(state[3:6])
        mass = state[6]

        # 1/m
        coef_mass = 1.0 / mass

        # g
        gravity_acceleration = self.asteroid.gravity_at_position(position)

        # w, w'
        angular_velocity, angular_acceleration = self.asteroid.angular_velocity_and_acceleration_at_time(time)

        # Fg, Fc
        thrust_acceleration = [0.0, 0.0, 0.0]
        for i in xrange(3):
            gravity_acceleration[i] *= coef_mass
            thrust_acceleration[i] = self.thrust[i] * coef_mass

        # w' x r
        euler_acceleration = cross_product(angular_acceleration, position)

        # w x (w x r)
        centrifugal_acceleration = cross_product(angular_velocity, cross_product(angular_velocity, position))

        # 2w x r'
        coriolis_acceleration = cross_product([2.0 * val for val in angular_velocity], velocity)

        for i in xrange(3):
            d_state_dt[i] = state[3+i]
            d_state_dt[3+i] = self.perturbations_acceleration[i] + gravity_acceleration[i] + thrust_acceleration[i]\
                              - coriolis_acceleration[i] - euler_acceleration[i] - centrifugal_acceleration[i]

        d_state_dt[6] = sqrt(self.thrust[0] * self.thrust[0]
                             + self.thrust[1] * self.thrust[1]
                             + self.thrust[2] * self.thrust[2]) * self.coef_earth_acceleration_mul_specific_impulse

        return d_state_dt
