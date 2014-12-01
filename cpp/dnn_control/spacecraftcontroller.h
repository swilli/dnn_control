#ifndef SPACECRAFTCONTROLLER_H
#define SPACECRAFTCONTROLLER_H

class SpacecraftController
{
public:
    SpacecraftController();

    void GetThrustForSensorData(double *sensor_data, double *thrust) const;
};

#endif // SPACECRAFTCONTROLLER_H
