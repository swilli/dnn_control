#ifndef VECTOR_H
#define VECTOR_H

#include <boost/array.hpp>
#include <math.h>
#include <sstream>

typedef boost::array<double,3> Vector3D;
typedef boost::array<double,2> Vector2D;

// c = u x v
inline Vector3D VectorCrossProduct(const Vector3D &vector_u, const Vector3D &vector_v) {
    Vector3D result = {vector_u[1] * vector_v[2] - vector_u[2] * vector_v[1],
                       vector_u[2] * vector_v[0] - vector_u[0] * vector_v[2],
                       vector_u[0] * vector_v[1] - vector_u[1] * vector_v[0]};
    return result;
}

inline void VectorCopy3D(const Vector3D &from, Vector3D &to) {
    to[0] = from[0];
    to[1] = from[1];
    to[2] = from[2];
}

inline void VectorCopy2D(const Vector2D &from, Vector2D &to) {
    to[0] = from[0];
    to[1] = from[1];
}

inline Vector3D VectorSub(const Vector3D &first, const Vector3D &second) {
    Vector3D result;
    result[0] = first[0] - second[0];
    result[1] = first[1] - second[1];
    result[2] = first[2] - second[2];
    return result;
}

inline double VectorNorm(const Vector3D &v) {
    double result = 0.0;
    for (unsigned int i = 0; i < 3; ++i) {
        result += v[i] * v[i];
    }
    return sqrt(result);
}

inline double VectorNorm(const Vector2D &v) {
    double result = 0.0;
    for (unsigned int i = 0; i < 2; ++i) {
        result += v[i] * v[i];
    }
    return sqrt(result);
}

inline std::string VectorToString(const Vector3D &v) {
    std::ostringstream oss;
    oss << "(";
    oss << v[0] << ",";
    oss << v[1] << ",";
    oss << v[2] << ")";
    return oss.str();
}

inline std::string VectorToString(const Vector2D &v) {
    std::ostringstream oss;
    oss << "(";
    oss << v[0] << ",";
    oss << v[1] << ")";
    return oss.str();
}

#endif // VECTOR_H
