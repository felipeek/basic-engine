#include "matrix.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <assert.h>
#include <light_array.h>

Matrix matrix_copy(const Matrix* m)
{
	Matrix copy;
	copy.data = malloc(m->rows * sizeof(r32*));
	for (u32 i = 0; i < m->rows; ++i)
	{
		copy.data[i] = malloc(m->columns * sizeof(r32));
		memcpy(copy.data[i], m->data[i], m->columns * sizeof(r32));
	}
	copy.rows = m->rows;
	copy.columns = m->columns;
	return copy;
}

Matrix matrix_create(u32 rows, u32 columns)
{
	Matrix m;
	m.data = malloc(rows * sizeof(r32*));
	for (u32 i = 0; i < rows; ++i)
	{
		m.data[i] = malloc(columns * sizeof(r32));
		memset(m.data[i], 0, columns * sizeof(r32));
	}
	m.rows = rows;
	m.columns = columns;
	return m;
}

void matrix_destroy(Matrix* m)
{
	for (u32 i = 0; i < m->rows; ++i)
		free(m->data[i]);
	free(m->data);
}

Matrix matrix_identity(u32 dimension)
{
	Matrix m = matrix_create(dimension, dimension);
	for (u32 i = 0; i < dimension; ++i)
		m.data[i][i] = 1.0f;
	return m;
}

void matrix_print(const Matrix* m)
{
	for (u32 r = 0; r < m->rows; ++r)
	{
		printf("[");
		for (u32 c = 0; c < m->columns; ++c)
		{
			printf("%.3f", m->data[r][c]);
			if (c != m->columns -1)
				printf(", ");
		}
		printf("]\n");
	}
	printf("\n");
}

Matrix matrix_transpose(const Matrix* m)
{
	Matrix transposed = matrix_create(m->columns, m->rows);

	for (u32 r = 0; r < m->rows; ++r)
		for (u32 c = 0; c < m->columns; ++c)
			transposed.data[c][r] = m->data[r][c];

	return transposed;
}

Matrix matrix_multiply(const Matrix* m1, const Matrix* m2)
{
	assert(m1->columns == m2->rows);
	Matrix result = matrix_create(m1->rows, m2->columns);
	for (u32 r = 0; r < m1->rows; ++r)
	{
		for (u32 c = 0; c < m2->columns; ++c)
		{
			r32 sum = 0;
			for (u32 i = 0; i < m1->columns; ++i)
				sum += m1->data[r][i] * m2->data[i][c];

			result.data[r][c] = sum;
		}
	}
	return result;
}

/* INPUT: A - array of pointers to rows of a square matrix having dimension N
 *        Tol - small tolerance number to detect failure when the matrix is near degenerate
 * OUTPUT: Matrix A is changed, it contains a copy of both matrices L-E and U as A=(L-E)+U such that P*A=L*U.
 *        The permutation matrix is not stored as a matrix, but in an integer vector P of size N+1 
 *        containing column indexes where the permutation matrix has "1". The last element P[N]=S+N, 
 *        where S is the number of row exchanges needed for determinant computation, det(P)=(-1)^S    
 */
static int LUPDecompose(r32 **A, int N, r32 Tol, int *P) {

    int i, j, k, imax; 
    r32 maxA, *ptr, absA;

    for (i = 0; i <= N; i++)
        P[i] = i; //Unit permutation matrix, P[N] initialized with N

    for (i = 0; i < N; i++) {
        maxA = 0.0;
        imax = i;

        for (k = i; k < N; k++)
            if ((absA = fabsf(A[k][i])) > maxA) { 
                maxA = absA;
                imax = k;
            }

        if (maxA < Tol) return 0; //failure, matrix is degenerate

        if (imax != i) {
            //pivoting P
            j = P[i];
            P[i] = P[imax];
            P[imax] = j;

            //pivoting rows of A
            ptr = A[i];
            A[i] = A[imax];
            A[imax] = ptr;

            //counting pivots starting from N (for determinant)
            P[N]++;
        }

        for (j = i + 1; j < N; j++) {
            A[j][i] /= A[i][i];

            for (k = i + 1; k < N; k++)
                A[j][k] -= A[j][i] * A[i][k];
        }
    }

    return 1;  //decomposition done 
}

/* INPUT: A,P filled in LUPDecompose; b - rhs vector; N - dimension
 * OUTPUT: x - solution vector of A*x=b
 */
static void LUPSolve(r32 **A, int *P, const r32 *b, int N, r32 *x) {

    for (int i = 0; i < N; i++) {
        x[i] = b[P[i]];

        for (int k = 0; k < i; k++)
            x[i] -= A[i][k] * x[k];
    }

    for (int i = N - 1; i >= 0; i--) {
        for (int k = i + 1; k < N; k++)
            x[i] -= A[i][k] * x[k];

        x[i] /= A[i][i];
    }
}

/* INPUT: A,P filled in LUPDecompose; N - dimension
 * OUTPUT: IA is the inverse of the initial matrix
 */
static void LUPInvert(r32 **A, int *P, int N, r32 **IA) {
  
    for (int j = 0; j < N; j++) {
        for (int i = 0; i < N; i++) {
            IA[i][j] = P[i] == j ? 1.0 : 0.0;

            for (int k = 0; k < i; k++)
                IA[i][j] -= A[i][k] * IA[k][j];
        }

        for (int i = N - 1; i >= 0; i--) {
            for (int k = i + 1; k < N; k++)
                IA[i][j] -= A[i][k] * IA[k][j];

            IA[i][j] /= A[i][i];
        }
    }
}

/* INPUT: A,P filled in LUPDecompose; N - dimension. 
 * OUTPUT: Function returns the determinant of the initial matrix
 */
static r32 LUPDeterminant(r32 **A, int *P, int N) {

    r32 det = A[0][0];

    for (int i = 1; i < N; i++)
        det *= A[i][i];

    return (P[N] - N) % 2 == 0 ? det : -det;
}

r32 matrix_determinant(const Matrix* m)
{
	assert(m->rows == m->columns);
	Matrix tmp = matrix_copy(m);
	int P[m->rows + 1];
	if (!LUPDecompose(tmp.data, m->rows, 0.00001f, P))
	{
		printf("Unable to get determinant of matrix! TOL failed.\n");
		matrix_destroy(&tmp);
		return 0.0f;
	}
	r32 d = LUPDeterminant(tmp.data, P, m->rows);
	matrix_destroy(&tmp);
	return d;
}

s32 matrix_invert(const Matrix* m, Matrix* out)
{
	assert(m->rows == m->columns);
	Matrix tmp = matrix_copy(m);
	int P[m->rows + 1];
	if (!LUPDecompose(tmp.data, m->rows, 0.00001f, P))
	{
		matrix_destroy(&tmp);
		return -1;
	}
	*out = matrix_create(m->rows, m->columns);
	LUPInvert(tmp.data, P, m->rows, out->data);
	matrix_destroy(&tmp);
	return 0;
}

s32 matrix_solve_system(const Matrix* m, const r32* b, r32* out)
{
	assert(m->rows == m->columns);
	Matrix tmp = matrix_copy(m);
	int P[m->rows + 1];
	if (!LUPDecompose(tmp.data, m->rows, 0.00000001f, P))
	{
		matrix_destroy(&tmp);
		return -1;
	}
	LUPSolve(tmp.data, P, b, m->rows, out);
	matrix_destroy(&tmp);
	return 0;
}

Matrix matrix_from_vec(r32* v)
{
	Matrix m = matrix_create(array_length(v), 1);
	for (u32 i = 0; i < array_length(v); ++i) {
		m.data[i][0] = v[i];
	}
	return m;
}

Matrix matrix_from_vec3(vec3 v)
{
	Matrix m = matrix_create(3, 1);
	m.data[0][0] = v.x;
	m.data[1][0] = v.y;
	m.data[2][0] = v.z;
	return m;
}

Matrix matrix_scalar_multiply(const Matrix* m, r32 scalar)
{
	Matrix result = matrix_copy(m);
	for (u32 i = 0; i < m->rows; ++i)
		for (u32 j = 0; j < m->columns; ++j)
			result.data[i][j] = result.data[i][j] * scalar;

	return result;
}

Matrix matrix_sum(const Matrix* m1, const Matrix* m2)
{
	assert(m1->rows == m2->rows);
	assert(m1->columns == m2->columns);

	Matrix copy = matrix_create(m1->rows, m1->columns);
	for (u32 i = 0; i < m1->rows; ++i)
		for (u32 j = 0; j < m1->columns; ++j)
			copy.data[i][j] = m1->data[i][j] + m2->data[i][j];

	return copy;
}