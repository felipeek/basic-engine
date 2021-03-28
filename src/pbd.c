#include "pbd.h"
#include <light_array.h>

static void solve_bend_constraint(Particle_Object* po, Constraint* c) {
	Particle* p1 = &po->particles[c->bend_constraint.p1];
	Particle* p2 = &po->particles[c->bend_constraint.p2];
	Particle* p3 = &po->particles[c->bend_constraint.p3];
	Particle* p4 = &po->particles[c->bend_constraint.p4];

	vec3 p2_pos = gm_vec3_subtract(p2->candidate_position, p1->candidate_position);
	vec3 p3_pos = gm_vec3_subtract(p3->candidate_position, p1->candidate_position);
	vec3 p4_pos = gm_vec3_subtract(p4->candidate_position, p1->candidate_position);

	vec3 p2p3 = gm_vec3_cross(p2_pos, p3_pos);
	vec3 p2p4 = gm_vec3_cross(p2_pos, p4_pos);
	r32 p2p3_length = gm_vec3_length(p2p3);
	r32 p2p4_length = gm_vec3_length(p2p4);
	vec3 n1 = gm_vec3_scalar_product(1.0f / p2p3_length, p2p3);
	vec3 n2 = gm_vec3_scalar_product(1.0f / p2p4_length, p2p4);
	r32 d =	MAX(MIN(gm_vec3_dot(n1, n2), 1.0f), -1.0f);
	vec3 p2n2 = gm_vec3_cross(p2_pos, n2);
	vec3 n1p2 = gm_vec3_cross(n1, p2_pos);
	vec3 p2n1 = gm_vec3_cross(p2_pos, n1);
	vec3 n2p2 = gm_vec3_cross(n2, p2_pos);
	vec3 p3n2 = gm_vec3_cross(p3_pos, n2);
	vec3 n1p3 = gm_vec3_cross(n1, p3_pos);
	vec3 p4n1 = gm_vec3_cross(p4_pos, n1);
	vec3 n2p4 = gm_vec3_cross(n2, p4_pos);
	vec3 q3 = gm_vec3_scalar_product(1.0f / p2p3_length, gm_vec3_add(p2n2, gm_vec3_scalar_product(d, n1p2)));
	vec3 q4 = gm_vec3_scalar_product(1.0f / p2p4_length, gm_vec3_add(p2n1, gm_vec3_scalar_product(d, n2p2)));
	vec3 q2_first_term = gm_vec3_scalar_product(-1.0f / p2p3_length, gm_vec3_add(p3n2, gm_vec3_scalar_product(d, n1p3)));
	vec3 q2_second_term = gm_vec3_scalar_product(-1.0f / p2p4_length, gm_vec3_add(p4n1, gm_vec3_scalar_product(d, n2p4)));
	vec3 q2 = gm_vec3_add(q2_first_term, q2_second_term);
	vec3 q1 = gm_vec3_add(gm_vec3_scalar_product(-1.0f, q2), gm_vec3_add(gm_vec3_scalar_product(1.0f, q3), gm_vec3_scalar_product(-1.0f, q4)));
	r32 q1_len = gm_vec3_length(q1);
	r32 q2_len = gm_vec3_length(q2);
	r32 q3_len = gm_vec3_length(q3);
	r32 q4_len = gm_vec3_length(q4);

	r32 den = p1->inverse_mass * q1_len * q1_len + p2->inverse_mass * q2_len * q2_len + p3->inverse_mass * q3_len * q3_len +
		p4->inverse_mass * q4_len * q4_len;
	if (den == 0.0f) {
		return;
	}
	vec3 delta_p1 = gm_vec3_scalar_product(-p1->inverse_mass * sqrtf(1.0f - d * d) * (acosf(d) - c->bend_constraint.phi0) / den, q1);
	vec3 delta_p2 = gm_vec3_scalar_product(-p2->inverse_mass * sqrtf(1.0f - d * d) * (acosf(d) - c->bend_constraint.phi0) / den, q2);
	vec3 delta_p3 = gm_vec3_scalar_product(-p3->inverse_mass * sqrtf(1.0f - d * d) * (acosf(d) - c->bend_constraint.phi0) / den, q3);
	vec3 delta_p4 = gm_vec3_scalar_product(-p4->inverse_mass * sqrtf(1.0f - d * d) * (acosf(d) - c->bend_constraint.phi0) / den, q4);

	r32 k = 1.0f - powf(1.0f - c->stiffness, 1.0f / 30.0f);
	p1->candidate_position = gm_vec3_add(p1->candidate_position, gm_vec3_scalar_product(k, delta_p1));
	p2->candidate_position = gm_vec3_add(p2->candidate_position, gm_vec3_scalar_product(k, delta_p2));
	p3->candidate_position = gm_vec3_add(p3->candidate_position, gm_vec3_scalar_product(k, delta_p3));
	p4->candidate_position = gm_vec3_add(p4->candidate_position, gm_vec3_scalar_product(k, delta_p4));
}

static void solve_distance_constraint(Particle_Object* po, Constraint* c) {
	Particle* p1 = &po->particles[c->distance_constraint.p1];
	Particle* p2 = &po->particles[c->distance_constraint.p2];
	if (p1->inverse_mass == 0.0f && p2->inverse_mass == 0.0f) {
		return;
	}
	vec3 p1p2 = gm_vec3_subtract(p1->candidate_position, p2->candidate_position);
	r32 p1p2_length = gm_vec3_length(p1p2);
	vec3 f = gm_vec3_scalar_product(p1p2_length - c->distance_constraint.distance, gm_vec3_scalar_product(1.0f / p1p2_length, p1p2));
	vec3 delta_p1 = gm_vec3_scalar_product(-p1->inverse_mass / (p1->inverse_mass + p2->inverse_mass), f);
	vec3 delta_p2 = gm_vec3_scalar_product(p2->inverse_mass / (p1->inverse_mass + p2->inverse_mass), f);
	r32 k = 1.0f - powf(1.0f - c->stiffness, 1.0f / 30.0f);
	p1->candidate_position = gm_vec3_add(p1->candidate_position, gm_vec3_scalar_product(k, delta_p1));
	p2->candidate_position = gm_vec3_add(p2->candidate_position, gm_vec3_scalar_product(k, delta_p2));
}

void pbd_simulate(Particle_Object* po, r32 dt) {
	if (dt <= 0.0f) {
		return;
	}

	vec3 force = gm_vec3_scalar_product(10.0f, gm_vec3_normalize((vec3){0.0f, 0.0f, 1.0f}));
	for (u32 i = 0; i < array_length(po->particles); ++i) {
		Particle* p = &po->particles[i];
		vec3 velocity_diff = gm_vec3_scalar_product(dt * p->inverse_mass, force);
		p->velocity = gm_vec3_add(p->velocity, velocity_diff);
		p->velocity = gm_vec3_scalar_product(0.999f, p->velocity);
		p->candidate_position = gm_vec3_add(p->position, gm_vec3_scalar_product(dt, p->velocity));
	}

	for (u32 i = 0; i < 5; ++i) {
		for (u32 j = 0; j < array_length(po->constraints); ++j) {
			Constraint* c = &po->constraints[j];
			switch (c->type) {
				case CONSTRAINT_DISTANCE: {
					solve_distance_constraint(po, c);
				} break;
				case CONSTRAINT_BEND: {
					solve_bend_constraint(po, c);
				} break;
			}
		}
	}

	for (u32 i = 0; i < array_length(po->particles); ++i) {
		Particle* p = &po->particles[i];
		p->velocity = gm_vec3_scalar_product(1.0f / dt, gm_vec3_subtract(p->candidate_position, p->position));
		p->position = p->candidate_position;
	}
}