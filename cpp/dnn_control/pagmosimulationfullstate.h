#ifndef PAGMOSIMULATIONFULLSTATE_H
#define PAGMOSIMULATIONFULLSTATE_H

#include "vector.h"
#include "samplefactory.h"
#include "asteroid.h"
#include "odesystem.h"
#include "sensorsimulatorfullstate.h"
#include "controllerfullstate.h"

#include <boost/tuple/tuple.hpp>

class PaGMOSimulationFullState
{
public:
    PaGMOSimulationFullState(const PaGMOSimulationFullState &other);

    PaGMOSimulationFullState(const unsigned int &random_seed, const double &simulation_time);
    PaGMOSimulationFullState(const unsigned int &random_seed, const double &simulation_time, const std::vector<double> &pid_coefficients);

    virtual ~PaGMOSimulationFullState();

    PaGMOSimulationFullState& operator=(const PaGMOSimulationFullState &other);

    boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > Evaluate();

    boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> > EvaluateDetailed();

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

    SampleFactory sample_factory_;

    Asteroid asteroid_;

    SensorSimulatorFullState *sensor_simulator_;

    ControllerFullState *controller_;

    SystemState initial_system_state_;
};

#endif // PAGMOSIMULATIONFULLSTATE_H
