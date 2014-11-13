def cross_product(vector_a, vector_b):
	vector_c = [vector_a[1] * vector_b[2] - vector_a[2] * vector_b[1], vector_a[2] * vector_b[0] - vector_a[0] * vector_b[2], vector_a[0] * vector_b[1] - vector_a[1] * vector_b[0]]
	return vector_c