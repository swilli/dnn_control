#include "asteroid.h"

#include <boost/tuple/tuple.hpp>
#include <boost/python.hpp>

namespace bp = boost::python;

class BoostAsteroid {
public:
    BoostAsteroid(const bp::list &semi_axis, const double &density, const bp::list &angular_velocity_xz, const double &time_bias);
    ~BoostAsteroid();
    
    bp::list GravityAccelerationAtPosition(const bp::list &position) const;

    bp::tuple AngularVelocityAndAccelerationAtTime(const double &time) const;

    bp::tuple NearestPointOnSurfaceToPosition(const bp::list &position) const;

    bp::list IntersectLineToCenterFromPosition(const bp::list &position) const;

    bp::list SemiAxis() const;

    double AngularVelocityPeriod() const;

private:
    // cpp Asteroid
    Asteroid *asteroid_cpp_;
};
