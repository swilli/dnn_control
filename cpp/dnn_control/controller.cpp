#include "controller.h"

Controller::Controller(const unsigned int &dimensions, const unsigned int &num_parameters, const double &maximum_thrust)
    : dimensions_(dimensions), number_of_parameters_(num_parameters), maximum_thrust_(maximum_thrust), maximum_thrust_per_dimension_(1.0 / sqrt(3) * maximum_thrust) {

}

Controller::Controller(const unsigned int &dimensions, const double &maximum_thrust)
    : dimensions_(dimensions), number_of_parameters_(0), maximum_thrust_(maximum_thrust), maximum_thrust_per_dimension_(1.0 / sqrt(3) * maximum_thrust) {

}

Controller::~Controller() {

}

unsigned int Controller::Dimensions() const {
    return dimensions_;
}

unsigned int Controller::NumberOfParameters() const {
    return number_of_parameters_;
}
