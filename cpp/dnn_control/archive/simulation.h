#ifndef SIMULATION_H
#define SIMULATION_H

#include "asteroid.h"
#include "vector.h"
#include "systemstate.h"

#define SIM_TEST_FOR_ORBIT  0

class Simulation {
public:
    Simulation(const unsigned int &random_seed);

    virtual ~Simulation();

    virtual boost::tuple<std::vector<double>, std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D>, std::vector<Vector3D> >  Evaluate() = 0;

    Asteroid& AsteroidOfSystem();

    class Exception {};
    class SizeMismatchException : public Exception {};

protected:
    class Observer {
    public:
        Observer(double &time) : time_(time){}
        void operator () (const SystemState &, const double &current_time) {
            time_ = current_time;
        }
    private:
        double &time_;
    };

    unsigned int random_seed_;

    double spacecraft_engine_noise_;

    double spacecraft_specific_impulse_;

    double spacecraft_minimum_mass_;

    double spacecraft_maximum_thrust_;

    double simulation_time_;

    double interaction_interval_;

    double perturbation_mean_;

    double perturbation_noise_;

    Asteroid asteroid_;

    SystemState initial_system_state_;

    Vector3D target_position_;

    std::vector<double> controller_parameters_;
};

#endif // SIMULATION_H
