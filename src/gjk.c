#include "gjk.h"
#include "float.h"
#include <light_array.h>
#include <stdio.h>
	
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

Minkowski_Point support(vec2 direction, Mesh m1, Mesh m2) {
	vec2 p1 = get_farthest_point_in_direction(direction, m1);
	vec2 p2 = get_farthest_point_in_direction((vec2){-direction.x, -direction.y}, m2);

	Minkowski_Point mp;
	mp.p = gm_vec2_subtract(p1, p2);
	mp.o_m1 = p1;
	mp.o_m2 = p2;
	return mp;
}

static void simplex_add(Simplex* simplex, Minkowski_Point mp) {
	switch (simplex->num) {
		case 0: {
			simplex->mp[0] = mp;
		} break;
		case 1: {
			simplex->mp[1] = mp;
		} break;
		case 2: {
			simplex->mp[2] = mp;
		} break;
		default: assert(0);
	}

	++simplex->num;
}

static void simplex_remove(Simplex* simplex, u32 index) {
	switch (index) {
		case 0: {
			simplex->mp[0] = simplex->mp[1];
			simplex->mp[1] = simplex->mp[2];
		} break;
		case 1: {
			simplex->mp[1] = simplex->mp[2];
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
		vec2 b = simplex->mp[0].p;
		vec2 c = simplex->mp[1].p;

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
		vec2 b = simplex->mp[0].p;
		vec2 ab = gm_vec2_subtract(b, a);
		vec2 ab_perp = triple_product(ab, ao, ab);
		*direction = ab_perp;
	}

	return false;
}

boolean gjk(Mesh m1, Mesh m2, Simplex* _simplex) {
	Simplex simplex = {0};
	vec2 direction = (vec2){1.0f, -1.0f};
	simplex_add(&simplex, support(direction, m1, m2));

	// negate the direction, so we search in the opposite direction
	direction = (vec2){-direction.x, -direction.y};

	while (true) {
		Minkowski_Point new_simplex_point = support(direction, m1, m2);
		simplex_add(&simplex, new_simplex_point);

		if (gm_vec2_dot(new_simplex_point.p, direction) <= 0.0f) {
			return false;
		} else {
			if (contains_origin(&simplex, &direction, new_simplex_point.p)) {
				if (_simplex) *_simplex = simplex;
				return true;
			}
		}
	}
}

typedef struct {
	r32 distance;
	vec2 normal;
	u32 index;
	vec2 collision_point;
} Edge;

static Edge find_closest_edge(Minkowski_Point* simplex) {
	Edge closest;
	closest.distance = FLT_MAX;

	for (u32 i = 0; i <	array_length(simplex); ++i) {
		int j = i + 1 == array_length(simplex) ? 0 : i + 1;
		vec2 a = simplex[i].p;
		vec2 b = simplex[j].p;
		vec2 e = gm_vec2_subtract(b, a);
		vec2 oa = a;
		vec2 n = triple_product(e, oa, e);
		n = gm_vec2_normalize(n);
		r32 d = gm_vec2_dot(n, a);
		if (d < closest.distance) {
			closest.distance = d;
			closest.normal = n;
			closest.index = j;

			vec2 ao = (vec2){-a.x, -a.y};
			r32 s = gm_vec2_dot(ao, e) / gm_vec2_dot(e, e);
			vec2 edge_in_world_space = gm_vec2_subtract(simplex[j].o_m1, simplex[i].o_m1);
			closest.collision_point = gm_vec2_add(simplex[i].o_m1, gm_vec2_scalar_product(s, edge_in_world_space));
		}
	}

	return closest;
}

vec2 epa(Simplex gjk_simplex, Mesh m1, Mesh m2, vec2* collision_point) {
	assert(gjk_simplex.num == 3);
	Minkowski_Point* simplex = array_new_len(Minkowski_Point, 3);
	array_push(simplex, gjk_simplex.mp[0]);
	array_push(simplex, gjk_simplex.mp[1]);
	array_push(simplex, gjk_simplex.mp[2]);
	
	const u32 LIMIT = 100;
	for (u32 i = 0; i < LIMIT; ++i) {
		Edge e = find_closest_edge(simplex);
		Minkowski_Point mp = support(e.normal, m1, m2);
		r32 d = gm_vec2_dot(mp.p, e.normal);

		if (d - e.distance < 0.00001f) {
			*collision_point = e.collision_point;
			e = find_closest_edge(simplex);
			array_free(simplex);
			return gm_vec2_scalar_product(d, gm_vec2_normalize(e.normal));
		} else {
			array_insert(simplex, mp, e.index);
		}
	}

	printf("EPA did not converge.\n");
	return (vec2){0.0f, 0.0f};
}