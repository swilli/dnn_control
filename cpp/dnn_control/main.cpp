#include "sensordatagenerator.h"
#include "evolutionaryrobotics.h"
#include "leastsquarespolicyrobotics.h"

#include "samplefactory.h"
#include "asteroid.h"
#include "constants.h"

int main(int argc, char *argv[]) {

    TrainNeuralNetworkController();

    const unsigned int num_asteroids = 10000;
    const unsigned int num_tests = 1000;

    SampleFactory sf(time(0));
    double error = 0.0;
    for (unsigned int i = 0; i < num_asteroids; ++i) {
        const double c_semi_axis = sf.SampleUniformReal(100.0, 8000.0);
        const double b_semi_axis_n = sf.SampleUniformReal(1.1, 2.0);
        const double a_semi_axis_n = sf.SampleUniformReal(1.1 * b_semi_axis_n, 4.0);
        const Vector3D &semi_axis = {a_semi_axis_n * c_semi_axis, b_semi_axis_n * c_semi_axis, c_semi_axis};
        const double density = sf.SampleUniformReal(1500.0, 3000.0);
        const double magn_angular_velocity = 0.85 * sqrt((kGravitationalConstant * 4.0/3.0 * kPi * semi_axis[0] * semi_axis[1] * semi_axis[2] * density) / (semi_axis[0] * semi_axis[0] * semi_axis[0]));
        const Vector2D &angular_velocity_xz = {sf.SampleSign() * sf.SampleUniformReal(magn_angular_velocity * 0.5, magn_angular_velocity), sf.SampleSign() * sf.SampleUniformReal(magn_angular_velocity * 0.5, magn_angular_velocity)};
        const double time_bias = sf.SampleUniformReal(0.0, 12.0 * 60 * 60);
        Asteroid asteroid(semi_axis, density, angular_velocity_xz, time_bias);

        for (unsigned int j = 0; j < num_tests; ++j) {
            const boost::tuple<Vector3D, double, double, double> sample = sf.SamplePointOutSideEllipsoid(semi_axis, 1.0, 1.0);
            const Vector3D &position = boost::get<0>(sample);
            const double &correct_phi = boost::get<1>(sample);
            const double &correct_theta = boost::get<2>(sample);

            const boost::tuple<double, double> coords = asteroid.LatitudeAndLongitudeAtPosition(position);
            const double &comp_phi = boost::get<0>(coords);
            const double &comp_theta = boost::get<1>(coords);

            double error_phi = correct_phi - comp_phi;
            error_phi = (error_phi < 0.0 ? -error_phi : error_phi);
            double error_theta = correct_theta - comp_theta;
            error_theta = (error_theta < 0.0 ? -error_theta : error_theta);

            error += error_phi + error_theta;
        }
    }

    std::cout << error / (num_asteroids * num_tests) << std::endl;
    return 0;
}

