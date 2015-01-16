#include "fixedsimulation.h"
#include "datacollector.h"
#include "odeint.h"

FixedSimulation::FixedSimulation(const unsigned int &random_seed) : Simulation(random_seed) {
    fixed_step_size_ = 0.1;
}

FixedSimulation::~FixedSimulation() {
}

boost::tuple<std::vector<double>, std::vector<Vector3D>, std::vector<Vector3D> > FixedSimulation::Evaluate() {
    std::vector<double> time_points;
    std::vector<Vector3D> evaluated_positions;
    std::vector<Vector3D> evaluated_heights;

    DataCollector collector(asteroid_, time_points, evaluated_positions, evaluated_heights);
    SystemState system_state(initial_system_state_);

    odeint::runge_kutta4<SystemState> stepper;
    try {
         integrate_const(stepper , system_, system_state, 0.0, simulation_time_, fixed_step_size_, collector);
    } catch (const Asteroid::Exception &exception) {
        std::cout << "The spacecraft crashed into the asteroid's surface." << std::endl;
    } catch (const ODESystem::Exception &exception) {
         std::cout << "The spacecraft is out of fuel." << std::endl;
    }

    return boost::make_tuple(time_points, evaluated_positions, evaluated_heights);
}

double FixedSimulation::FixedStepSize() const {
    return fixed_step_size_;
}


