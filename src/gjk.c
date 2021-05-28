#include "gjk.h"
#include "float.h"
#include <light_array.h>

typedef struct {
	vec2 p[3];
	u32 num;
} Simplex;
	
vec2 get_farthest_point_in_direction(vec2 direction, Mesh m) {
	vec2 point = m.vertices[0].position;
	r32 max = gm_vec2_dot(point, direction);

	for (u32 i = 1; i < array_length(m.vertices); ++i) {
		Vertex* v = &m.vertices[i];
		r32 dot = gm_vec2_dot(v->position, direction);
		if (dot > max) {
			max = dot;
			point = v->position;
		}
	}

	return point;
}

vec2 support(vec2 direction, Mesh m1, Mesh m2) {
	vec2 p1 = get_farthest_point_in_direction(direction, m1);
	vec2 p2 = get_farthest_point_in_direction((vec2){-direction.x, -direction.y}, m2);
	return gm_vec2_subtract(p1, p2);
}

static void simplex_add(Simplex* simplex, vec2 p) {
	switch (simplex->num) {
		case 0: {
			simplex->p[0] = p;
		} break;
		case 1: {
			simplex->p[1] = p;
		} break;
		case 2: {
			simplex->p[2] = p;
		} break;
		default: assert(0);
	}

	++simplex->num;
}

static void simplex_remove(Simplex* simplex, u32 index) {
	switch (index) {
		case 0: {
			simplex->p[0] = simplex->p[1];
			simplex->p[1] = simplex->p[2];
		} break;
		case 1: {
			simplex->p[1] = simplex->p[2];
		} break;
		case 2: {
		} break;
		default: assert(0);
	}

	--simplex->num;
}

static vec2 triple_product(vec2 a, vec2 b, vec2 c) {
	return gm_vec2_subtract(gm_vec2_scalar_product(gm_vec2_dot(c, a), b), gm_vec2_scalar_product(gm_vec2_dot(c, b), a));
}

static boolean contains_origin(Simplex* simplex, vec2* direction, vec2 last_simplex_point) {
	vec2 a = last_simplex_point;
	vec2 ao = (vec2){-last_simplex_point.x, -last_simplex_point.y};
	if (simplex->num == 3) {
		vec2 b = simplex->p[0];
		vec2 c = simplex->p[1];

		vec2 ab = gm_vec2_subtract(b, a);
		vec2 ac = gm_vec2_subtract(c, a);

		vec2 ab_perp = triple_product(ac, ab, ab);
		vec2 ac_perp = triple_product(ab, ac, ac);

		if (gm_vec2_dot(ab_perp, ao) > 0.0f) {
			simplex_remove(simplex, 1);
			*direction = ab_perp;
		} else {
			if (gm_vec2_dot(ac_perp, ao) > 0.0f) {
				simplex_remove(simplex, 0);
				*direction = ac_perp;
			} else {
				return true;
			}
		}
	} else {
		vec2 b = simplex->p[0];
		vec2 ab = gm_vec2_subtract(b, a);
		vec2 ab_perp = triple_product(ab, ao, ab);
		*direction = ab_perp;
	}

	return false;
}

boolean gjk(Mesh m1, Mesh m2) {
	Simplex simplex = {0};
	vec2 direction = (vec2){1.0f, -1.0f};
	simplex_add(&simplex, support(direction, m1, m2));

	// negate the direction, so we search in the opposite direction
	direction = (vec2){-direction.x, -direction.y};

	while (true) {
		vec2 new_simplex_point = support(direction, m1, m2);
		simplex_add(&simplex, new_simplex_point);

		if (gm_vec2_dot(new_simplex_point, direction) <= 0.0f) {
			return false;
		} else {
			if (contains_origin(&simplex, &direction, new_simplex_point)) {
				return true;
			}
		}
	}
}