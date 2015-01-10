#include "controller5d.h"

Controller5D::Controller5D(const double &maximum_thrust) : Controller(5, maximum_thrust) {
}

Controller5D::~Controller5D() {

}

Vector3D Controller5D::GetThrustForSensorData(const SensorData &sensor_data) {
    Vector3D thrust;
    thrust[0] = 0.0;
    thrust[1] = 0.0;
    thrust[2] = 0.0;
    return thrust;
}
