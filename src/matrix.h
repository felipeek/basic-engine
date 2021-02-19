#ifndef BASIC_ENGINE_MATRIX_H
#define BASIC_ENGINE_MATRIX_H
#include "common.h"
#include "gm.h"
typedef struct {
	u32 rows;
	u32 columns;
	r32** data;
} Matrix;

Matrix matrix_create(u32 rows, u32 columns);
Matrix matrix_copy(const Matrix* m);
void matrix_destroy(Matrix* m);
void matrix_print(const Matrix* m);
Matrix matrix_transpose(const Matrix* m);
Matrix matrix_multiply(const Matrix* m1, const Matrix* m2);
Matrix matrix_invert(const Matrix* m);
Matrix matrix_from_vec3(vec3 v);
r32 matrix_determinant(const Matrix* m);
#endif