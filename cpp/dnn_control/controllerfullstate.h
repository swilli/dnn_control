#ifndef FULLSTATECONTROLLER_H
#define FULLSTATECONTROLLER_H

#include "controller.h"
#include "configuration.h"

class ControllerFullState : public Controller {
public:
    static const unsigned int kDimensions;

    ControllerFullState(const double &maximum_thrust, const Vector3D &target_position);
    virtual ~ControllerFullState();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, some RL solution, a NN, ...
    virtual Vector3D GetThrustForSensorData(const SensorData &sensor_data);

    void SetCoefficients(const std::vector<double> &pid_coefficients);

private:

    double latest_control_action_;

    std::vector<double>  target_state_;

    std::vector<double> pid_coefficients_;

    std::vector<double> previous_error_;

    std::vector<double> integral_;
};

#endif // FULLSTATECONTROLLER_H
