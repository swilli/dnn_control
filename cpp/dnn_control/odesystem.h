#ifndef ODESYSTEM_H
#define ODESYSTEM_H

#include "asteroid.h"
#include "vector.h"

#include <boost/array.hpp>
#include <boost/random.hpp>
#include <boost/random/normal_distribution.hpp>

typedef boost::array<double,7> State;

class ODESystem {
    /*
     * This class represents the dynamics in a rotating reference frame (which we have around an asteroid) using ordinary differential equations.
     * Implementation is inspired from "Control of Hovering Spacecraft Using Altimetry" by S. Sawai et. al.
     */
public:
    ODESystem(Asteroid asteroid, const double &control_noise);

    // This operator gets called by the boost stepper to integrate the system.
    // The function computes d/dt x = f(x, t). Or in our case: "d_state_dt" = this(state, time)
    void operator()(const State &state, State &d_state_dt, const double &time) ;

    // The asteroid which is at the center of the system
    Asteroid asteroid_;

    // Isp (for deltaV budget computation)
    double spacecraft_specific_impulse_;

    // control_distribution_ ~ N(0, control_noise), whereas perturbation_noise is given in the constructor
    boost::variate_generator<boost::mt19937, boost::normal_distribution<> > control_distribution_;

    // T for each dimension
    Vector3D thrust_;

    // We assume some random noise in the system which is given by "perturbations_acceleration_" and is constant during one integration step
    Vector3D perturbations_acceleration_;

    // The spacecraft state [x,y,z,dx,dy,dz,m]
    State state_;
};

#endif // ODESYSTEM_H
