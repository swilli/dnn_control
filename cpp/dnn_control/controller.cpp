#include "controller.h"

Controller::Controller(const unsigned int &dimensions, const double &maximum_thrust) : dimensions_(dimensions), maximum_thrust_(maximum_thrust) {

}

Controller::~Controller() {

}

unsigned int Controller::Dimensions() const {
    return dimensions_;
}
