#ifndef PAGMOSIMULATIONFULLSTATE_H
#define PAGMOSIMULATIONFULLSTATE_H

#include "vector.h"
#include "asteroid.h"
#include "systemstate.h"

#include <boost/tuple/tuple.hpp>

class PaGMOSimulationFullState {
public:
    PaGMOSimulationFullState(const unsigned int &random_seed, const double &simulation_time);
    PaGMOSimulationFullState(const unsigned int &random_seed, const double &simulation_time, const std::vector<double> &pid_coefficients);

    virtual ~PaGMOSimulationFullState();


    boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > Evaluate();

    boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluateDetailed();

    boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluateImpl2();

    double FixedStepSize() const;

    double MinimumStepSize() const;

    Asteroid& AsteroidOfSystem();

    class Exception {};
    class SizeMismatchException : public Exception {};

private:
    void Init();

    unsigned int random_seed_;

    double simulation_time_;

    double minimum_step_size_;

    double fixed_step_size_;

    double engine_noise_;

    double perturbation_noise_;

    double spacecraft_specific_impulse_;

    double spacecraft_maximum_thrust_;

    double interaction_interval_;

    Asteroid asteroid_;

    SystemState initial_system_state_;

    Vector3D target_position_;

    std::vector<double> full_state_coefficients_;
};

#endif // PAGMOSIMULATIONFULLSTATE_H
