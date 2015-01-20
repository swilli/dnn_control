#ifndef DATACOLLECTOR_H
#define DATACOLLECTOR_H

#include "asteroid.h"
#include "systemstate.h"

#include <vector>

class DataCollector {
public:
    DataCollector(Asteroid &asteroid, std::vector<double> &simulated_time, std::vector<Vector3D> &positions, std::vector<Vector3D> &heights, std::vector<Vector3D> &velocities, std::vector<double> &masses);
    ~DataCollector();

    void operator () (const SystemState &system_state , const double &current_time);

private:
    std::vector<double> &time_points_;
    std::vector<Vector3D>  &positions_;
    std::vector<Vector3D> &heights_;
    std::vector<Vector3D> &velocities_;
    std::vector<double> &masses_;

    Asteroid &asteroid_;
};

#endif // DATACOLLECTOR_H
