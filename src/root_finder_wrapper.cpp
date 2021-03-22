#include <root_finder.h>
#include <light_array.h>
#include "common.h"

extern "C" r32* find_roots(r32* polynomial_coefficients) {
	Eigen::VectorXd coeffs(array_length(polynomial_coefficients));
	for (u32 i = 0; i < array_length(polynomial_coefficients); ++i) {
		coeffs[i] = polynomial_coefficients[i];
	}
	std::set<double> all_roots = RootFinder::solvePolynomial(coeffs, -100.0, 100.0, 0.00001);
	r32* results = array_new(r32);
	std::set<double>::iterator it;
	for (it = all_roots.begin(); it != all_roots.end(); ++it) {
		array_push(results, *it);
	}

	return results;
}