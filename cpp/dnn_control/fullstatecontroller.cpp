#include "fullstatecontroller.h"

FullStateController::FullStateController(const Vector3D &target_position) : SpacecraftController(7) {
    for (int i = 0; i < 3; ++i) {
        target_position_[i] = target_position[i];
    }
}

FullStateController::~FullStateController() {

}

void FullStateController::GetThrustForSensorData(const SensorData &sensor_data, Vector3D &thrust) {

}
