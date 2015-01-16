#include "datacollector.h"


DataCollector::DataCollector(Asteroid &asteroid, std::vector<double> &simulated_time, std::vector<Vector3D> &positions, std::vector<Vector3D> &heights) : asteroid_(asteroid), time_points_(simulated_time), positions_(positions), heights_(heights) {

}

DataCollector::~DataCollector() {

}

void DataCollector::operator ()(const SystemState &system_state, const double &current_time) {
    time_points_.push_back(current_time);

    // Compute spacecraft position
    const Vector3D position = {system_state[0], system_state[1], system_state[2]};

    // Compute height
    const Vector3D surf_pos = boost::get<0>(asteroid_.NearestPointOnSurfaceToPosition(position));
    const Vector3D height = {position[0] - surf_pos[0], position[1] - surf_pos[1], position[2] - surf_pos[2]};

    positions_.push_back(position);
    heights_.push_back(height);
}

