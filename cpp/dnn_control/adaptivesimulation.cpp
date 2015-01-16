#include "adaptivesimulation.h"
#include "datacollector.h"
#include "controlledrungekutta.h"
#include "odeint.h"

AdaptiveSimulation::AdaptiveSimulation(const unsigned int &random_seed) : Simulation(random_seed) {
    minimum_step_size_ = 0.1;
}

AdaptiveSimulation::~AdaptiveSimulation() {

}

boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > AdaptiveSimulation::Evaluate() {
    std::vector<double> time_points;
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_heights;

    DataCollector collector(asteroid_, time_points, evaluated_positions, evaluated_heights);
    SystemState system_state(initial_system_state_);

    typedef odeint::runge_kutta_cash_karp54<SystemState> ErrorStepper;
    typedef odeint::modified_controlled_runge_kutta<ErrorStepper> ControlledStepper;
    ControlledStepper controlled_stepper;

    try {
        integrate_adaptive(controlled_stepper , system_, system_state, 0.0, simulation_time_, minimum_step_size_, collector);
    } catch (const Asteroid::Exception &exception) {
        std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    }

    return boost::make_tuple(time_points, evaluated_positions, evaluated_heights);
}

double AdaptiveSimulation::MinimumStepSize() const {
    return minimum_step_size_;
}
