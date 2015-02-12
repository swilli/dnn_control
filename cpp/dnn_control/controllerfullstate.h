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

#if CFS_WITH_VELOCITY
typedef boost::array<double,6> Array;
#else
typedef boost::array<double,3> Array;
#endif

    double latest_control_action_;

    Array pid_coefficients_;

    Array target_state_;

    Array previous_error_;

    Array integral_;
};

#endif // FULLSTATECONTROLLER_H
