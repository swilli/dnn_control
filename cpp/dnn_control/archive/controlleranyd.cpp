#include "controlleranyd.h"

ControllerAnyD::ControllerAnyD(const double &control_frequency, const double &maximum_thrust) : Controller(CONTROLLER_SENSOR_DATA_DIMENSIONS, maximum_thrust) {

}

ControllerAnyD::~ControllerAnyD() {

}

Vector3D ControllerAnyD::GetThrustForSensorData(const SensorData &sensor_data) {
    Vector3D thrust;
    thrust[0] = 0.0;
    thrust[1] = 0.0;
    thrust[2] = 0.0;
    return thrust;
}

