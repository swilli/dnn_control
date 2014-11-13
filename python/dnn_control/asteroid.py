class Asteroid:

    # inertia_x < inertia_y < inertia_z
    # Implementation based on Landau Lifshitz Mechanics eq 37.8 - 37.10
    def __init__(self, inertia_x, inertia_y, inertia_z, density, angular_velocity, time_bias):
        from math import sqrt

        self.density = float(density)
        self.inertia_x = float(inertia_x)
        self._inertia_x_pow2 = inertia_x ** 2
        self.inertia_y = float(inertia_y)
        self._inertia_y_pow2 = inertia_y ** 2
        self.inertia_z = float(inertia_z)
        self._inertia_z_pow2 = inertia_z ** 2
        self.time_bias = float(time_bias)

        angular_velocity = [float(val) for val in angular_velocity]

        sgnx = (-1.0, 1.0)[angular_velocity[0] > 0]
        sgny = (-1.0, 1.0)[angular_velocity[0] * angular_velocity[2] > 0]
        sgnz = (-1.0, 1.0)[angular_velocity[2] > 0]

        self.energy_mul2 = self.inertia_x * angular_velocity[0] ** 2 + self.inertia_y * angular_velocity[1] ** 2 + self.inertia_z * angular_velocity[2] ** 2
        self.momentum_pow2 = self.inertia_x ** 2 * angular_velocity[0] ** 2 + self.inertia_y ** 2 * angular_velocity[1] ** 2 + self.inertia_z ** 2 * angular_velocity[2] ** 2

        inertia_y = self.inertia_y
        self._inversion = False
        if self.momentum_pow2 < self.energy_mul2 * inertia_y:
            inertia_z = self.inertia_x
            inertia_x = self.inertia_z
            self._inversion = True
            sgnz = (-1.0, 1.0)[angular_velocity[0] > 0]
            sgnx = (-1.0, 1.0)[angular_velocity[2] > 0]


        self.elliptic_coef_angular_velocity_x = sgnx * sqrt((self.energy_mul2 * inertia_z - self.momentum_pow2) / (inertia_x * (inertia_z - inertia_x)))
        self.elliptic_coef_angular_velocity_y = sgny * sqrt((self.energy_mul2 * inertia_z - self.momentum_pow2) / (inertia_y * (inertia_z - inertia_y)))
        self.elliptic_coef_angular_velocity_z = sgnz * sqrt((self.momentum_pow2 - self.energy_mul2 * inertia_x) / (inertia_z * (inertia_z - inertia_x)))

        self.elliptic_tau = sqrt((inertia_z - inertia_y) * (self.momentum_pow2 - self.energy_mul2 * inertia_x) / (inertia_x * inertia_y * inertia_z))
        self.elliptic_modulus = (inertia_y - inertia_x) * (self.energy_mul2 * inertia_z - self.momentum_pow2) / ((inertia_z - inertia_y) * (self.momentum_pow2 - self.energy_mul2 * inertia_x))

        self._cached_angular_velocity_time = -1
        self._cached_angular_velocity = [0.0, 0.0, 0.0]

    # Ported from "Ellipsoid_Gravity_Field.m" written by Dario Cersosimo,
    # October 16, 2009.
    def gravity_at_position(self, position):
        from math import asin, sqrt
        from numpy import array, roots
        from constants import PI, GRAVITATIONAL_CONSTANT
        from ellipsoidgravityfield import legendre_elliptic_integral_pf, legendre_elliptic_integral_pe

        acceleration = [0, 0, 0]
        error_tolerance = 1e-10

        density = self.density
        inertia_x = self.inertia_x
        inertia_y = self.inertia_y
        inertia_z = self.inertia_z
        inertia_x_pow2 = self._inertia_x_pow2
        inertia_y_pow2 = self._inertia_y_pow2
        inertia_z_pow2 = self._inertia_z_pow2

        pos_z = position[2]
        pos_z_pow_2 = pos_z ** 2
        pos_x_pow_2 = position[0] ** 2
        pos_y_pow_2 = position[1] ** 2

        coef_2 = -(pos_x_pow_2 + pos_y_pow_2 + pos_z_pow_2 -
                   inertia_x_pow2 - inertia_y_pow2 - inertia_z_pow2)
        coef_1 = -inertia_z_pow2 * (pos_x_pow_2 + pos_y_pow_2) + inertia_y_pow2 * (
            inertia_z_pow2 - pos_x_pow_2 - pos_z_pow_2) + inertia_x_pow2 * (inertia_y_pow2 + inertia_z_pow2 - pos_y_pow_2 - pos_z_pow_2)
        coef_0 = -inertia_y_pow2 * inertia_z_pow2 * pos_x_pow_2 + inertia_x_pow2 * \
            (-inertia_z_pow2 * pos_y_pow_2 + inertia_y_pow2 *
             (inertia_z - pos_z) * (inertia_z + pos_z))
        polynomial = array([1.0, coef_2, coef_1, coef_0])
        maximum_root = max(roots(polynomial))

        phi = asin(
            sqrt((inertia_x_pow2 - inertia_z_pow2) / (maximum_root + inertia_x_pow2)))
        k = sqrt((inertia_x_pow2 - inertia_y_pow2) /
                 (inertia_x_pow2 - inertia_z_pow2))

        integral_f1 = legendre_elliptic_integral_pf(phi, k, error_tolerance)
        integral_e1 = legendre_elliptic_integral_pe(phi, k, error_tolerance)

        fac_1 = 4.0 * PI * GRAVITATIONAL_CONSTANT * density * inertia_x * \
            inertia_y * inertia_z / sqrt(inertia_x_pow2 - inertia_z_pow2)
        fac_2 = sqrt((inertia_x_pow2 - inertia_z_pow2) / ((inertia_x_pow2 + maximum_root)
                                                          * (inertia_y_pow2 + maximum_root) * (inertia_z_pow2 + maximum_root)))

        acceleration[
            0] = fac_1 / (inertia_x_pow2 - inertia_y_pow2) * (integral_e1 - integral_f1)
        acceleration[1] = fac_1 * ((inertia_z_pow2 - inertia_x_pow2) * integral_e1 / ((inertia_x_pow2 - inertia_y_pow2) * (inertia_y_pow2 - inertia_z_pow2)
                                                                                      ) + integral_f1 / (inertia_x_pow2 - inertia_y_pow2) + (inertia_z_pow2 + maximum_root) * fac_2 / (inertia_y_pow2 - inertia_z_pow2))
        acceleration[2] = fac_1 * ((inertia_y_pow2 + maximum_root)
                                   * fac_2 - integral_e1) / (inertia_z_pow2 - inertia_y_pow2)
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
            self._cached_angular_velocity = [self.elliptic_coef_angular_velocity_z * dn_tau, self.elliptic_coef_angular_velocity_y * sn_tau, self.elliptic_coef_angular_velocity_x * cn_tau]
        else:
            self._cached_angular_velocity = [self.elliptic_coef_angular_velocity_x * cn_tau, self.elliptic_coef_angular_velocity_y * sn_tau, self.elliptic_coef_angular_velocity_z * dn_tau]
        return self._cached_angular_velocity

    # compute d/dt w
    def angular_acceleration_at_time(self, time):
        angular_velocity = self.angular_velocity_at_time(time)
        inertia_x = self.inertia_x
        inertia_y = self.inertia_y
        inertia_z = self.inertia_z
        return [(inertia_z - inertia_y) * angular_velocity[2] * angular_velocity[1] / inertia_x, (inertia_x - inertia_z) * angular_velocity[0] * angular_velocity[2] / inertia_y, (inertia_y - inertia_x) * angular_velocity[1] * angular_velocity[0] / inertia_z]
