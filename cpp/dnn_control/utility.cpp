void CrossProduct(double *vector_u, double *vector_v, double *crossed) {
    crossed[0] = vector_u[1] * vector_v[2] - vector_u[2] * vector_v[1];
    crossed[1] = vector_u[2] * vector_v[0] - vector_u[0] * vector_v[2];
    crossed[2] = vector_u[0] * vector_v[1] - vector_u[1] * vector_v[0];
}
