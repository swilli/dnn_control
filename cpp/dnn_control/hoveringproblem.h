#ifndef HOVERINGPROBLEM_H
#define HOVERINGPROBLEM_H

#include "simulator.h"

class HoveringProblem
{
public:
    HoveringProblem();
    ~HoveringProblem();

    void Init(const unsigned int &random_seed, const bool &full_state_controlled);
    void CreateVisualizationFile(const std::string &path_to_trajectory_file);

private:
    double simulation_time_;

    Simulator *simulator_;
};

#endif // HOVERINGPROBLEM_H
