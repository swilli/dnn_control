#include "datacollector.h"


DataCollector::DataCollector(const Asteroid &asteroid, std::vector<double> &simulated_time, std::vector<double> &masses, std::vector<Vector3D> &positions, std::vector<Vector3D> &heights, std::vector<Vector3D> &velocities, const bool &collect_height)
    : time_points_(simulated_time), masses_(masses), positions_(positions), heights_(heights), velocities_(velocities), asteroid_(asteroid), collect_height_(collect_height) {

}

DataCollector::~DataCollector() {

}

void DataCollector::operator ()(const SystemState &system_state, const double &current_time) {
    if (fmod(current_time, 20.0)) {
        return;
    }
    // Get current time
    time_points_.push_back(current_time);

    // Get spacecraft mass
    const double mass = system_state[6];
    masses_.push_back(mass);

    // Get spacecraft position
    const Vector3D position = {system_state[0], system_state[1], system_state[2]};
    positions_.push_back(position);

    // Get spacecraft velocity
    const Vector3D velocity = {system_state[3], system_state[4], system_state[5]};
    velocities_.push_back(velocity);


    // Compute height
    if (collect_height_) {
        const Vector3D surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
        const Vector3D height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};
        heights_.push_back(height);
    }
}

