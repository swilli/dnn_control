#ifndef VECTOR_H
#define VECTOR_H

#include <boost/array.hpp>

typedef boost::array<double,3> Vector3D;
typedef boost::array<double,2> Vector2D;

inline void VectorCopy3D(const Vector3D &from, Vector3D &to) {
    to[0] = from[0];
    to[1] = from[1];
    to[2] = from[2];
}

inline void VectorCopy2D(const Vector2D &from, Vector2D &to) {
    to[0] = from[0];
    to[1] = from[1];
}

#endif // VECTOR_H
