#ifndef DATACOLLECTOR_H
#define DATACOLLECTOR_H

#include "asteroid.h"
#include "systemstate.h"

#include <vector>

class DataCollector {
public:
    DataCollector(Asteroid &asteroid, std::vector<double> &simulated_time, std::vector<double> &masses, std::vector<Vector3D> &positions, std::vector<Vector3D> &heights, std::vector<Vector3D> &velocities, const bool &collect_height=true);
    ~DataCollector();

    void operator () (const SystemState &system_state , const double &current_time);

private:
    std::vector<double> &time_points_;
    std::vector<double> &masses_;
    std::vector<Vector3D>  &positions_;
    std::vector<Vector3D> &heights_;
    std::vector<Vector3D> &velocities_;

    Asteroid &asteroid_;

    bool collect_height_;
};

#endif // DATACOLLECTOR_H
