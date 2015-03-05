#ifndef VECTOR_H
#define VECTOR_H

#include <boost/array.hpp>
#include <cmath>
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

// s  = first - second
inline Vector3D VectorSub(const Vector3D &first, const Vector3D &second) {
    Vector3D result;
    result[0] = first[0] - second[0];
    result[1] = first[1] - second[1];
    result[2] = first[2] - second[2];
    return result;
}

inline Vector3D VectorAdd(const Vector3D &first, const Vector3D &second) {
    Vector3D result;
    result[0] = first[0] + second[0];
    result[1] = first[1] + second[1];
    result[2] = first[2] + second[2];
    return result;
}

// n = ||vector||
inline double VectorNorm(const Vector3D &vector) {
    double result = 0.0;
    for (unsigned int i = 0; i < 3; ++i) {
        result += vector[i] * vector[i];
    }
    return sqrt(result);
}

// n = ||vector||
inline double VectorNorm(const Vector2D &vector) {
    double result = 0.0;
    for (unsigned int i = 0; i < 2; ++i) {
        result += vector[i] * vector[i];
    }
    return sqrt(result);
}

// nv = vector/||vector||
inline Vector3D VectorNormalize(const Vector3D &vector) {
    Vector3D result;
    const double norm = VectorNorm(vector);
    if (norm == 0) {
        return Vector3D(vector);
    }
    const double scal_norm = 1.0 / norm;
    result[0] = vector[0] * scal_norm;
    result[1] = vector[1] * scal_norm;
    result[2] = vector[2] * scal_norm;
    return result;
}

// nv = vector/||vector||
inline Vector2D VectorNormalize(const Vector2D &vector) {
    Vector2D result;
    const double norm = VectorNorm(vector);
    if (norm == 0) {
        return Vector2D(vector);
    }
    const double scal_norm = 1.0 / norm;
    result[0] = vector[0] * scal_norm;
    result[1] = vector[1] * scal_norm;
    return result;
}

// String representation of vector
inline std::string VectorToString(const Vector3D &vector) {
    std::ostringstream oss;
    oss << "[";
    oss << vector[0] << ",";
    oss << vector[1] << ",";
    oss << vector[2] << "]";
    return oss.str();
}

// String representation of vector
inline std::string VectorToString(const Vector2D &vector) {
    std::ostringstream oss;
    oss << "[";
    oss << vector[0] << ",";
    oss << vector[1] << "]";
    return oss.str();
}

// p = <first, second>
inline double VectorDotProduct(const Vector3D &first, const Vector3D &second) {
    return first[0] * second[0] + first[1] * second[1] + first[2] * second[2];
}

// p = <first, second>
inline double VectorDotProduct(const Vector2D &first, const Vector2D &second) {
    return first[0] * second[0] + first[1] * second[1];
}

#endif // VECTOR_H
