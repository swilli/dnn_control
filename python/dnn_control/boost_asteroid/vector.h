#ifndef VECTOR_H
#define VECTOR_H

typedef double Vector3D[3];
typedef double Vector2D[2];

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
