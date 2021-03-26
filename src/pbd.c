#include "pbd.h"
#include <light_array.h>

static void solve_distance_constraint(Particle_Object* po, Constraint* c) {
	Particle* p1 = &po->particles[c->distance_constraint.p1];
	Particle* p2 = &po->particles[c->distance_constraint.p2];
	if (p1->inverse_mass == 0.0f && p2->inverse_mass == 0.0f) {
		return;
	}
	vec3 p1p2 = gm_vec3_subtract(p1->candidate_position, p2->candidate_position);
	r32 p1p2_length = gm_vec3_length(p1p2);
	vec3 f = gm_vec3_scalar_product(p1p2_length - c->distance_constraint.distance, gm_vec3_scalar_product(1.0f / p1p2_length, p1p2));
	r32 k = 1.0f - powf(1.0f - c->stiffness, 1.0f / 30.0f);
	vec3 delta_p1 = gm_vec3_scalar_product(k, gm_vec3_scalar_product(-p1->inverse_mass / (p1->inverse_mass + p2->inverse_mass), f));
	vec3 delta_p2 = gm_vec3_scalar_product(k, gm_vec3_scalar_product(p2->inverse_mass / (p1->inverse_mass + p2->inverse_mass), f));
	p1->candidate_position = gm_vec3_add(p1->candidate_position, delta_p1);
	p2->candidate_position = gm_vec3_add(p2->candidate_position, delta_p2);
}

void pbd_simulate(Particle_Object* po, r32 dt) {
	if (dt <= 0.0f) {
		return;
	}

	vec3 force = gm_vec3_scalar_product(10.0f, gm_vec3_normalize((vec3){1.0f, -3.0f, 1.0f}));
	for (u32 i = 0; i < array_length(po->particles); ++i) {
		Particle* p = &po->particles[i];
		vec3 velocity_diff = gm_vec3_scalar_product(dt * p->inverse_mass, force);
		p->velocity = gm_vec3_add(p->velocity, velocity_diff);
		p->velocity = gm_vec3_scalar_product(0.999f, p->velocity);
		p->candidate_position = gm_vec3_add(p->position, gm_vec3_scalar_product(dt, p->velocity));
	}

	for (u32 i = 0; i < 30; ++i) {
		for (u32 j = 0; j < array_length(po->constraints); ++j) {
			Constraint* c = &po->constraints[j];
			switch (c->type) {
				case CONSTRAINT_DISTANCE: {
					solve_distance_constraint(po, c);
				}
			}
		}
	}

	for (u32 i = 0; i < array_length(po->particles); ++i) {
		Particle* p = &po->particles[i];
		p->velocity = gm_vec3_scalar_product(1.0f / dt, gm_vec3_subtract(p->candidate_position, p->position));
		p->position = p->candidate_position;
	}
}