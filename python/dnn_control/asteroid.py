class Asteroid:

    """
    This class represents the dynamics and gravity of an asteroid shaped as a rigid ellipsoid having different
    rotational properties along its three principal inertia axis (Ix, Iy, Iz). 

    For the angular velocity dynamics the analytical solution in terms of Jacobi elliptic functions is used (Implementation
    largely inspired from Landau Lifshitz Mechanics paragrpah 37). We thus assume w_y(-t_bias) = 0. So we only are free to specify two
    more initial conditions and we do so by assigning kinetic energy and angular momentum. As a result the user is not free to specify
    the initial angular velocity, but only the values of the two prime integrals. He does so by defining an angular velocity vector in
    the constructor, this is used only to compute kinetic energy and rotational momentum, the second of its components (w_y) will thus be disregarded
    as 0 will instead be used.

    For the asteroid gravity, we use the implementation of Dario Cersosimo whi kindly sent us his matlab files.
    """

    def __init__(self, inertia_x, inertia_y, inertia_z, density, angular_velocity, time_bias):

        """
        This constructs the asteroid object from its inertia properties and its prime integrals value

        USAGE: ast = Asteroid(0.1,0.2,0.3,2.3,[0.23,0.1,0.56],234)

        * inertia_x, inertia_y, inertia_z:     Moments of inertia Ix < Iy < Iz (UNITS)
        * density:                             Asteroid density (UNITS)
        * angular_velocity:                    Asteroid hypothetical angular velocity (used to define prime integrals only)
        * time_bias:                           A time bias applied so that w_y(-t_bias) = 0

        """
        # SU controls (Stupid User)
        if (inertia_x == inertia_y) or (inertia_x == inertia_z) or (inertia_y == inertia_z):
            raise Exception("This function does not work for symmetric tops, make sure Ix, Iy and Iz are different")
        if sorted([inertia_x, inertia_y, inertia_z]) != [inertia_x, inertia_y, inertia_z]:
            raise Exception("This function assumes Ix < Iy < Iz, please make sure this is the case")
        if angular_velocity[0] == 0 or angular_velocity[2] == 0:
            raise Exception("You cannot define zero values for w_x or w_z as this "
                            "functions already assumes w_y=0 (thus the motion would be a rotation")

        from math import sqrt

        self.density = float(density)
        self.inertia_x = float(inertia_x)
        self._inertia_x_pow2 = inertia_x ** 2
        self.inertia_y = float(inertia_y)
        self._inertia_y_pow2 = inertia_y ** 2
        self.inertia_z = float(inertia_z)
        self._inertia_z_pow2 = inertia_z ** 2
        self.time_bias = float(time_bias)

        self._angular_velocity = [float(val) for val in angular_velocity]

        sign_x = (-1.0, 1.0)[self._angular_velocity[0] > 0]
        sign_y = (-1.0, 1.0)[self._angular_velocity[0] * self._angular_velocity[2] > 0]
        sign_z = (-1.0, 1.0)[self._angular_velocity[2] > 0]

        self.energy_mul2 = self.inertia_x * self._angular_velocity[0] ** 2 \
                           + self.inertia_y * self._angular_velocity[1] ** 2 \
                           + self.inertia_z * self._angular_velocity[2] ** 2
        self.momentum_pow2 = self.inertia_x ** 2 * self._angular_velocity[0] ** 2 \
                             + self.inertia_y ** 2 * self._angular_velocity[1] ** 2\
                             + self.inertia_z ** 2 * self._angular_velocity[2] ** 2

        inertia_y = self.inertia_y
        self._inversion = False
        if self.momentum_pow2 < self.energy_mul2 * inertia_y:
            inertia_z = self.inertia_x
            inertia_x = self.inertia_z
            self._inversion = True
            sign_z = (-1.0, 1.0)[self._angular_velocity[0] > 0]
            sign_x = (-1.0, 1.0)[self._angular_velocity[2] > 0]


        self.elliptic_coef_angular_velocity_x = sign_x * sqrt((self.energy_mul2 * inertia_z - self.momentum_pow2)
                                                              / (inertia_x * (inertia_z - inertia_x)))
        self.elliptic_coef_angular_velocity_y = sign_y * sqrt((self.energy_mul2 * inertia_z - self.momentum_pow2)
                                                              / (inertia_y * (inertia_z - inertia_y)))
        self.elliptic_coef_angular_velocity_z = sign_z * sqrt((self.momentum_pow2 - self.energy_mul2 * inertia_x)
                                                              / (inertia_z * (inertia_z - inertia_x)))

        self.elliptic_tau = sqrt((inertia_z - inertia_y) * (self.momentum_pow2 - self.energy_mul2 * inertia_x)
                                 / (inertia_x * inertia_y * inertia_z))
        self.elliptic_modulus = (inertia_y - inertia_x) * (self.energy_mul2 * inertia_z - self.momentum_pow2) \
                                / ((inertia_z - inertia_y) * (self.momentum_pow2 - self.energy_mul2 * inertia_x))

        self._cached_angular_velocity_time = -1
        self._cached_angular_velocity = [0.0, 0.0, 0.0]


    def __str__(self):
        result = ["Asteroid:"]
        keys = sorted([key for key in self.__dict__])
        for key in keys:
            result.append("{key}='{value}'".format(key=key, value=self.__dict__[key]))

        return "\n ".join(result)

    # Ported from "Ellipsoid_Gravity_Field.m" written by Dario Cersosimo,
    # October 16, 2009.
    def gravity_at_position(self, position):
        from math import asin, sqrt
        from numpy import array, roots
        from constants import PI, GRAVITATIONAL_CONSTANT
        from ellipsoidgravityfield import legendre_elliptic_integral_pf, legendre_elliptic_integral_pe

        acceleration = [0, 0, 0]
        error_tolerance = 1e-10

        # since Ellipsoid_Gravity_Field.m assumes Ix > Iy > Iz, we have to rotate axis around y by 90 deg
        density = self.density
        inertia_z = self.inertia_x
        inertia_y = self.inertia_y
        inertia_x = self.inertia_z
        inertia_z_pow2 = self._inertia_x_pow2
        inertia_y_pow2 = self._inertia_y_pow2
        inertia_x_pow2 = self._inertia_z_pow2

        pos_z = -position[0]
        pos_z_pow_2 = pos_z ** 2
        pos_x_pow_2 = position[2] ** 2
        pos_y_pow_2 = position[1] ** 2

        coef_2 = -(pos_x_pow_2 + pos_y_pow_2 + pos_z_pow_2 -
                   inertia_x_pow2 - inertia_y_pow2 - inertia_z_pow2)
        coef_1 = -inertia_z_pow2 * (pos_x_pow_2 + pos_y_pow_2) \
                 + inertia_y_pow2 * (inertia_z_pow2 - pos_x_pow_2 - pos_z_pow_2) \
                 + inertia_x_pow2 * (inertia_y_pow2 + inertia_z_pow2 - pos_y_pow_2 - pos_z_pow_2)
        coef_0 = -inertia_y_pow2 * inertia_z_pow2 * pos_x_pow_2 + inertia_x_pow2 * \
            (-inertia_z_pow2 * pos_y_pow_2 + inertia_y_pow2 *
             (inertia_z - pos_z) * (inertia_z + pos_z))
        polynomial = array([1.0, coef_2, coef_1, coef_0])
        maximum_root = max(roots(polynomial))

        val_sin = (inertia_x_pow2 - inertia_z_pow2) / (maximum_root + inertia_x_pow2)
        if val_sin > 1.0:
            val_sin = 1.0
        elif val_sin < -1.0:
            val_sin = 1.0
        phi = asin(sqrt(val_sin))
        k = sqrt((inertia_x_pow2 - inertia_y_pow2) /
                 (inertia_x_pow2 - inertia_z_pow2))

        integral_f1 = legendre_elliptic_integral_pf(phi, k, error_tolerance)
        integral_e1 = legendre_elliptic_integral_pe(phi, k, error_tolerance)

        fac_1 = 4.0 * PI * GRAVITATIONAL_CONSTANT * density * inertia_x * \
            inertia_y * inertia_z / sqrt(inertia_x_pow2 - inertia_z_pow2)

        sum_z = inertia_z_pow2 + maximum_root
        if sum_z <= 0.0:
            sum_z = 1e-7

        fac_2 = sqrt((inertia_x_pow2 - inertia_z_pow2) / ((inertia_x_pow2 + maximum_root)
                                                          * (inertia_y_pow2 + maximum_root)
                                                          * sum_z))

        acceleration[0] = fac_1 / (inertia_x_pow2 - inertia_y_pow2) * (integral_e1 - integral_f1)
        acceleration[1] = fac_1 * ((inertia_z_pow2 - inertia_x_pow2) * integral_e1 /
                                   ((inertia_x_pow2 - inertia_y_pow2) * (inertia_y_pow2 - inertia_z_pow2))
                                   + integral_f1 / (inertia_x_pow2 - inertia_y_pow2) + (inertia_z_pow2 + maximum_root)
                                   * fac_2 / (inertia_y_pow2 - inertia_z_pow2))

        acceleration[2] = fac_1 * ((inertia_y_pow2 + maximum_root)
                                   * fac_2 - integral_e1) / (inertia_z_pow2 - inertia_y_pow2)

        # rotate back
        acceleration[0], acceleration[2] = -acceleration[2], acceleration[0]
        return acceleration

    # compute w
    def angular_velocity_at_time(self, time):
        from scipy.special import ellipj
        from math import fabs

        if fabs(time - self._cached_angular_velocity_time) < 1e-10:
            return self._cached_angular_velocity

        self._cached_angular_velocity_time = time
        time += self.time_bias
        time *= self.elliptic_tau
        sn_tau, cn_tau, dn_tau, _ = ellipj(time, self.elliptic_modulus)
        if self._inversion:
            self._cached_angular_velocity = [self.elliptic_coef_angular_velocity_z * dn_tau,
                                             self.elliptic_coef_angular_velocity_y * sn_tau,
                                             self.elliptic_coef_angular_velocity_x * cn_tau]
        else:
            self._cached_angular_velocity = [self.elliptic_coef_angular_velocity_x * cn_tau,
                                             self.elliptic_coef_angular_velocity_y * sn_tau,
                                             self.elliptic_coef_angular_velocity_z * dn_tau]
        return self._cached_angular_velocity

    # compute d/dt w
    def angular_acceleration_at_time(self, time):
        angular_velocity = self.angular_velocity_at_time(time)
        inertia_x = self.inertia_x
        inertia_y = self.inertia_y
        inertia_z = self.inertia_z
        return [(inertia_z - inertia_y) * angular_velocity[2] * angular_velocity[1] / inertia_x,
                (inertia_x - inertia_z) * angular_velocity[0] * angular_velocity[2] / inertia_y,
                (inertia_y - inertia_x) * angular_velocity[1] * angular_velocity[0] / inertia_z]
