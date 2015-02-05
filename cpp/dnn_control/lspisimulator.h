#ifndef LSPISIMULATOR_H
#define LSPISIMULATOR_H

#include "vector.h"

#include "systemstate.h"
#include "asteroid.h"
#include "samplefactory.h"

class LSPISimulator {
public:
    LSPISimulator(const unsigned int &random_seed);
    ~LSPISimulator();


    boost::tuple<SystemState, bool> NextState(const SystemState &state, const double &time, const Vector3D &thrust);

    Asteroid &AsteroidOfSystem();

    SampleFactory &GetSampleFactory();

    double InteractionInterval() const;

private:
    class Observer {
    public:
        Observer(double &time) : time_(time){}
        void operator () (const SystemState &, const double &current_time) {
            time_ = current_time;
        }
    private:
        double &time_;
    };

    double minimum_step_size_;

    double interaction_interval_;

    double spacecraft_engine_noise_;

    double spacecraft_specific_impulse_;

    double spacecraft_maximum_thrust_;

    double spacecraft_minimum_mass_;

    double spacecraft_maximum_mass_;

    double perturbation_mean_;

    double perturbation_noise_;

    Asteroid asteroid_;

    SampleFactory sample_factory_;
};

#endif // LSPISIMULATOR_H
