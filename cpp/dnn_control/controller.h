#ifndef SPACECRAFTCONTROLLER_H
#define SPACECRAFTCONTROLLER_H

#include "vector.h"
#include "sensorsimulator.h"

class Controller {
        /*
         * This abstract class generates the thrust for a given sensor_data input.
        */
public:
    Controller(const unsigned int &dimensions, const unsigned int &num_parameters, const double &maximum_thrust);
    Controller(const unsigned int &dimensions, const double &maximum_thrust);

    virtual ~Controller();

    // thrust = F(sensor_data), whereas F can be eg., a PD controller, a NN, ...
    virtual Vector3D GetThrustForSensorData(const SensorData &sensor_data) = 0;

    // The number of dimensions the controller works on in the function GetThrustForSensorData
    unsigned int Dimensions() const;

    // The number of parameters the controller can be changed with
    virtual unsigned int NumberOfParameters() const;

    // Controller can throw the following exceptions
    class Exception {};
    class SizeMismatchException : public Exception {};

protected:
    // How large can the sensor space be
    unsigned int dimensions_;

    // How many parameters does the controller have to tweak
    unsigned int number_of_parameters_;

    // What is the maximum absolute thrust that the spacecraft can generate
    double maximum_thrust_;

    // The maximum absolute thrust the spacecraft can generate with respect to one dimension (x,y,z)
    double maximum_thrust_per_dimension_;
};

#endif // SPACECRAFTCONTROLLER_H
