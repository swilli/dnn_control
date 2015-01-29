#include "boostasteroid.h"
#include "vector.h"
#include <iostream>

BoostAsteroid::BoostAsteroid(const bp::list &semi_axis, const double &density, const bp::list &angular_velocity_xz, const double &time_bias) {
    const Vector3D semi_axis_cpp = {bp::extract<double>(semi_axis[0]),
        bp::extract<double>(semi_axis[1]),
        bp::extract<double>(semi_axis[2])};

    const Vector2D angular_velocity_cpp_xz = {bp::extract<double>(angular_velocity_xz[0]),
        bp::extract<double>(angular_velocity_xz[1])};

    asteroid_cpp_ = new Asteroid(semi_axis_cpp, density, angular_velocity_cpp_xz, time_bias);
}

BoostAsteroid::~BoostAsteroid() {
    delete asteroid_cpp_;
}

bp::list BoostAsteroid::GravityAccelerationAtPosition(const bp::list &position) const {
    const Vector3D position_cpp = {bp::extract<double>(position[0]),
        bp::extract<double>(position[1]),
        bp::extract<double>(position[2])};

    const Vector3D gravity = asteroid_cpp_->GravityAccelerationAtPosition(position_cpp);

    bp::list gravity_py;
    gravity_py.append(gravity[0]);
    gravity_py.append(gravity[1]);
    gravity_py.append(gravity[2]);

    return gravity_py;
}

bp::tuple BoostAsteroid::AngularVelocityAndAccelerationAtTime(const double &time) const {
    const boost::tuple<Vector3D, Vector3D> result = asteroid_cpp_->AngularVelocityAndAccelerationAtTime(time);
    const Vector3D angular_velocity_cpp = boost::get<0>(result);
    const Vector3D angular_acceleration_cpp = boost::get<1>(result);

    bp::list angular_velocity_py;
    bp::list angular_acceleration_py;

    for (unsigned int i = 0; i < 3; ++i) {
        angular_velocity_py.append(angular_velocity_cpp[i]);
        angular_acceleration_py.append(angular_acceleration_cpp[i]);
    }

    return make_tuple(angular_velocity_py, angular_acceleration_py);
}

bp::tuple BoostAsteroid::NearestPointOnSurfaceToPosition(const bp::list &position) const {
    const Vector3D position_cpp = {bp::extract<double>(position[0]),
        bp::extract<double>(position[1]),
        bp::extract<double>(position[2])};

    const boost::tuple<Vector3D, double> result = asteroid_cpp_->NearestPointOnSurfaceToPosition(position_cpp);
    const Vector3D surface_point_cpp = boost::get<0>(result);
    const double distance = boost::get<1>(result);
    
    bp::list surface_point_py;
    surface_point_py.append(surface_point_cpp[0]);
    surface_point_py.append(surface_point_cpp[1]);
    surface_point_py.append(surface_point_cpp[2]);

    return make_tuple(surface_point_py, distance);
}

bp::list BoostAsteroid::SemiAxis() const {
    const Vector3D semi_axis_cpp = asteroid_cpp_->SemiAxis();

    bp::list semi_axis_py;
    semi_axis_py.append(semi_axis_cpp[0]);
    semi_axis_py.append(semi_axis_cpp[1]);
    semi_axis_py.append(semi_axis_cpp[2]);

    return semi_axis_py;
}

BOOST_PYTHON_MODULE(boost_asteroid)
{
    bp::class_<BoostAsteroid>("BoostAsteroid", bp::init<const bp::list &, const double &, const bp::list &, const double &>())
            .def("gravity_acceleration_at_position", &BoostAsteroid::GravityAccelerationAtPosition)
            .def("angular_velocity_and_acceleration_at_time", &BoostAsteroid::AngularVelocityAndAccelerationAtTime)
            .def("nearest_point_on_surface_to_position", &BoostAsteroid::NearestPointOnSurfaceToPosition)
            .def("semi_axis", &BoostAsteroid::SemiAxis)
            ;
}

