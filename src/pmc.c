#include "pmc.h"
#include "gjk.h"
#include <light_array.h>
#include "graphics.h"

//#define USE_RESERVE
#define IGNORE_PMC

int key_compare(const void* _key1, const void* _key2) {
	const PMC_Map_Key key1 = *(PMC_Map_Key*)_key1;
	const PMC_Map_Key key2 = *(PMC_Map_Key*)_key2;

	return (key1.e1 == key2.e1 && key1.e2 == key2.e2) || (key1.e1 == key2.e2 && key1.e2 == key2.e1);
}

unsigned int key_hash(const void* _key) {
	const PMC_Map_Key key = *(PMC_Map_Key*)_key;
	
	return (unsigned int)key.e1 + (unsigned int)key.e2;
}

void pmc_init() {
	assert(hash_map_create(&pmc_map, 32, sizeof(PMC_Map_Key), sizeof(PMC*), key_compare, key_hash) == 0);
}

void pmc_destroy(PMC* pmc) {
	// todo
}

void pmc_render(Perspective_Camera* camera) {
	PMC_Map_Key key;
	PMC* pmc;
	Hash_Map_Iterator iterator = hash_map_get_iterator(&pmc_map);
	while ((iterator = hash_map_iterator_next(&pmc_map, iterator, &key, &pmc)) != HASH_MAP_ITERATOR_END) {
		for (u32 i = 0; i < array_length(pmc->contacts); ++i) {
			PMC_Contact* contact = &pmc->contacts[i];
			mat3 rot_matrix = quaternion_get_matrix3(&contact->e2->world_rotation);
			vec3 collision_point = gm_vec3_add(gm_mat3_multiply_vec3(&rot_matrix, contact->r2_lc), contact->e2->world_position);
			//vec3 collision_point = gm_vec3_add(contact->e2->world_position, contact->r2_wc);
			//graphics_renderer_debug_points(&collision_point, 1, (vec4){1.0f, 1.0f, 1.0f, 1.0f});
			//graphics_renderer_debug_vector(collision_point,
			//	gm_vec3_add(collision_point, contact->normal), (vec4){1.0f, 1.0f, 1.0f, 1.0f});
			//collision_point = gm_vec3_add(contact->e1->world_position, contact->r1_wc);
			rot_matrix = quaternion_get_matrix3(&contact->e1->world_rotation);
			collision_point = gm_vec3_add(gm_mat3_multiply_vec3(&rot_matrix, contact->r1_lc), contact->e1->world_position);
			graphics_renderer_debug_points(&collision_point, 1, (vec4){1.0f, 1.0f, 1.0f, 1.0f});
			graphics_renderer_debug_vector(collision_point,
				gm_vec3_add(collision_point, contact->normal), (vec4){1.0f, 0.0f, 0.0f, 1.0f});
		}
	}
	graphics_renderer_primitives_flush(camera);
}

static vec3 find_ortho(vec3 v) {
	vec3 aux = v;
	aux.x += 100.0;
	aux = gm_vec3_normalize(aux);
	aux = gm_vec3_cross(v, aux);
	return gm_vec3_normalize(aux);
}

static boolean is_contact_still_valid(PMC_Contact contact) {
	mat3 q1_mat = quaternion_get_matrix3(&contact.e1->world_rotation);
	contact.r1_wc = gm_mat3_multiply_vec3(&q1_mat, contact.r1_lc);

	mat3 q2_mat = quaternion_get_matrix3(&contact.e2->world_rotation);
	contact.r2_wc = gm_mat3_multiply_vec3(&q2_mat, contact.r2_lc);

	vec3 collision_point1 = gm_vec3_add(contact.e1->world_position, contact.r1_wc);
	vec3 collision_point2 = gm_vec3_add(contact.e2->world_position, contact.r2_wc);
	r32 distance = -gm_vec3_dot(gm_vec3_subtract(collision_point1, collision_point2), contact.normal);

	// check distance
	if (distance > 0.00f) {
		printf("Contact not valid: Distance\n");
		return false;
	}

	// check d2d
	r32 d = gm_vec3_dot(gm_vec3_subtract(collision_point1, collision_point2), contact.normal);
	vec3 projected_point = gm_vec3_subtract(collision_point1, gm_vec3_scalar_product(d, contact.normal));
	vec3 projected_diff = gm_vec3_subtract(collision_point2, projected_point);
	r32 d2d = gm_vec3_dot(projected_diff, projected_diff);
	if (d2d > 0.001f) {
		printf("Contact not valid: D2D\n");
		return false;
	}

	return true;
}

void pmc_update() {
#ifdef IGNORE_PMC
	return;
#endif
	PMC_Map_Key key;
	PMC* pmc;
	Hash_Map_Iterator iterator = hash_map_get_iterator(&pmc_map);
	while ((iterator = hash_map_iterator_next(&pmc_map, iterator, &key, &pmc)) != HASH_MAP_ITERATOR_END) {
#ifdef USE_RESERVE
		for (int i = 0; i < array_length(pmc->reserve); ++i) {
			PMC_Contact* current = &pmc->reserve[i];
			if (is_contact_still_valid(*current)) {
				printf("adding back because contact is now valid!\n");
				array_remove(pmc->reserve, i);
				--i;
				pmc_add(*current);
			}
		}
#endif

		for (int i = 0; i < array_length(pmc->contacts); ++i) {
			PMC_Contact* current = &pmc->contacts[i];
			if (!is_contact_still_valid(*current)) {
				//printf("removing because contact not valid anymore\n");
				array_remove(pmc->contacts, i);
				--i;
#ifdef USE_RESERVE
				array_push(pmc->reserve, *current);
#endif
			}
		}
	}
}

static r32 calc_area_4_points(vec3 p0, vec3 p1, vec3 p2, vec3 p3) {
	vec3 a0 = gm_vec3_subtract(p0, p1);
	vec3 a1 = gm_vec3_subtract(p0, p2);
	vec3 a2 = gm_vec3_subtract(p0, p3);
	vec3 b0 = gm_vec3_subtract(p2, p3);
	vec3 b1 = gm_vec3_subtract(p1, p3);
	vec3 b2 = gm_vec3_subtract(p1, p2);

	vec3 tmp0 = gm_vec3_cross(a0, b0);
	vec3 tmp1 = gm_vec3_cross(a1, b1);
	vec3 tmp2 = gm_vec3_cross(a2, b2);

	r32 l0 = gm_vec3_length(tmp0);
	r32 l1 = gm_vec3_length(tmp1);
	r32 l2 = gm_vec3_length(tmp2);

	if (l0 > l1 && l0 > l2) return l0;
	if (l1 > l2) return l1;
	return l2;
}

static int get_cache_entry(PMC* pmc, vec3 cp) {
	int nearest_point = -1;
	r32 shortest_dist = 0.3f;

	for (int i = 0; i < array_length(pmc->contacts); ++i) {
		PMC_Contact* current = &pmc->contacts[i];
		vec3 collision_point2 = gm_vec3_add(current->e2->world_position, current->r2_wc);

		r32 diff2 = gm_vec3_length(gm_vec3_subtract(collision_point2, cp));
		if (diff2 < shortest_dist) {
			nearest_point = i;
			shortest_dist = diff2;
		}
	}

	return nearest_point;
}

static int sort_cached_points(PMC* pmc, PMC_Contact contact) {
	vec3 received_collision_point1 = gm_vec3_add(contact.e1->world_position, contact.r1_wc);
	vec3 received_collision_point2 = gm_vec3_add(contact.e2->world_position, contact.r2_wc);

	int max_penetration_index = -1;

	// calculate max penetration
	vec3* collision_points1 = array_new(vec3);
	vec3* collision_points2 = array_new(vec3);
	r32 deepest_penetration = gm_vec3_dot(gm_vec3_subtract(received_collision_point1, received_collision_point2), contact.normal);
	PMC_Contact chosen;
	int chosen_idx = -1;

	for (int i = 0; i < array_length(pmc->contacts); ++i) {
		PMC_Contact* current = &pmc->contacts[i];

		vec3 current_collision_point1 = gm_vec3_add(current->e1->world_position, current->r1_wc);
		vec3 current_collision_point2 = gm_vec3_add(current->e2->world_position, current->r2_wc);
		array_push(collision_points1, current_collision_point1);
		array_push(collision_points2, current_collision_point2);

		r32 current_penetration = gm_vec3_dot(gm_vec3_subtract(current_collision_point1, current_collision_point2), current->normal);

		if (current_penetration > deepest_penetration) {
			deepest_penetration = current_penetration;
			chosen_idx = i;
			chosen = *current;
		}
	}

	r32 res0 = 0.0f, res1 = 0.0f, res2 = 0.0f, res3 = 0.0f;
	if (chosen_idx != 0) {
		// todo: we have a pair of collision points! what to do with collision_points1?
		res0 = calc_area_4_points(received_collision_point2, collision_points2[1], collision_points2[2], collision_points2[3]);
	}

	if (chosen_idx != 1) {
		// todo: we have a pair of collision points! what to do with collision_points1?
		res1 = calc_area_4_points(received_collision_point2, collision_points2[0], collision_points2[2], collision_points2[3]);
	}

	if (chosen_idx != 2) {
		// todo: we have a pair of collision points! what to do with collision_points1?
		res2 = calc_area_4_points(received_collision_point2, collision_points2[0], collision_points2[1], collision_points2[3]);
	}

	if (chosen_idx != 3) {
		// todo: we have a pair of collision points! what to do with collision_points1?
		res3 = calc_area_4_points(received_collision_point2, collision_points2[0], collision_points2[1], collision_points2[2]);
	}

	if (res0 > res1 && res0 > res2 && res0 > res3) return 0;
	if (res1 > res2 && res1 > res3) return 1;
	if (res2 > res3) return 2;
	return 3;
}

static PMC* get_pmc_for_entity_pair(Entity* e1, Entity* e2) {
	PMC* pmc;
	PMC_Map_Key key;
	key.e1 = e1;
	key.e2 = e2;

	if (hash_map_get(&pmc_map, &key, &pmc)) {
		pmc = malloc(sizeof(PMC));
		pmc->contacts = array_new(PMC_Contact);
		pmc->reserve = array_new(PMC_Contact);
		assert(hash_map_put(&pmc_map, &key, &pmc) == 0);
	}

	return pmc;
}

void pmc_add(PMC_Contact contact) {
	PMC* pmc = get_pmc_for_entity_pair(contact.e1, contact.e2);
	//printf("size: %ld\n", array_length(pmc->reserve));

#ifdef IGNORE_PMC
	array_push(pmc->contacts, contact);
	return;
#endif

	if (!is_contact_still_valid(contact)) {
#ifdef USE_RESERVE
		array_push(pmc->reserve, contact);
#endif
		return;
	}

	vec3 received_collision_point2 = gm_vec3_add(contact.e2->world_position, contact.r2_wc);
	int existing_entry = get_cache_entry(pmc, received_collision_point2);

	if (existing_entry >= 0) {
		pmc->contacts[existing_entry] = contact;
		return;
	}

	if (array_length(pmc->contacts) < 4) {
		array_push(pmc->contacts, contact);
		return;
	}

	int insert_index = sort_cached_points(pmc, contact);
	pmc->contacts[insert_index] = contact;
}

void pmc_clear(Entity* e1, Entity* e2) {
	PMC* pmc = get_pmc_for_entity_pair(e1, e2);
	array_clear(pmc->contacts);
	array_clear(pmc->reserve);
}

void pmc_clear_all() {
	PMC_Map_Key key;
	PMC* pmc;
	Hash_Map_Iterator iterator = hash_map_get_iterator(&pmc_map);
	while ((iterator = hash_map_iterator_next(&pmc_map, iterator, &key, &pmc)) != HASH_MAP_ITERATOR_END) {
		array_clear(pmc->contacts);
		array_clear(pmc->reserve);
	}
}

void pmc_perturb(Entity* e1, Entity* e2, vec3 normal) {
	Quaternion e1_rotation = e1->world_rotation;
	Quaternion e2_rotation = e2->world_rotation;

	vec3 ortho = find_ortho(normal);

	const r32 PERTURBATION_ANGLE = 0.1f;
	const int NUM_ITERATIONS = 4;
	Quaternion Rp = quaternion_new(ortho, PERTURBATION_ANGLE);
	r32 angle = 360.0f / NUM_ITERATIONS;

	for (int i = 0; i < NUM_ITERATIONS; ++i) {
		Quaternion Rn = quaternion_new(normal, angle * i);
		Quaternion Rn_inv = quaternion_inverse(&Rn);
		Quaternion new_rotation = quaternion_product(&Rn, &e2_rotation);
		new_rotation = quaternion_product(&Rp, &new_rotation);
		new_rotation = quaternion_product(&Rn_inv, &new_rotation);
		graphics_entity_set_rotation(e2, new_rotation);

		graphics_entity_update_bounding_shapes(e1);
		graphics_entity_update_bounding_shapes(e2);
		GJK_Support_List gjk_sl = {0};
		Collision_Point cp1, cp2;
		if (collision_gjk_collides(&gjk_sl, &e1->bs, &e2->bs)) {
			boolean found_cp = collision_epa(gjk_sl.simplex, &e1->bs, &e2->bs, &cp1);
			gjk_sl = (GJK_Support_List){0};
			if (found_cp && collision_gjk_collides(&gjk_sl, &e2->bs, &e1->bs)) {
				found_cp = collision_epa(gjk_sl.simplex, &e2->bs, &e1->bs, &cp2);
				if (found_cp) {
					PMC_Contact contact = (PMC_Contact){0};
					contact.e1 = e1;
					contact.e2 = e2;
					contact.normal = gm_vec3_normalize(normal); // REVIEW, DO WE USE THIS NORMAL OR THE ORIGINAL ONE?

					// we calculate only r1_lc and r2_lc because the cube is artifically rotated
					vec3 r1_wc = gm_vec3_subtract(cp1.collision_point, e1->world_position);
					vec3 r2_wc = gm_vec3_subtract(cp2.collision_point, e2->world_position);

					Quaternion q1_inv = quaternion_inverse(&e1->world_rotation);
					mat3 q1_mat = quaternion_get_matrix3(&q1_inv);
					contact.r1_lc = gm_mat3_multiply_vec3(&q1_mat, r1_wc);

					Quaternion q2_inv = quaternion_inverse(&e2->world_rotation);
					mat3 q2_mat = quaternion_get_matrix3(&q2_inv);
					contact.r2_lc = gm_mat3_multiply_vec3(&q2_mat, r2_wc);

					// now we calculate r1_wc and r2_wc based on the real rotation
					q1_mat = quaternion_get_matrix3(&e1_rotation);
					contact.r1_wc = gm_mat3_multiply_vec3(&q1_mat, contact.r1_lc);

					q2_mat = quaternion_get_matrix3(&e2_rotation);
					contact.r2_wc = gm_mat3_multiply_vec3(&q2_mat, contact.r2_lc);

					graphics_entity_set_rotation(e1, e1_rotation); // reset for safety
					graphics_entity_set_rotation(e2, e2_rotation); // reset for safety
					pmc_add(contact);
					//printf("perturbation find collision!\n");
				}
			}
		}
	}

	graphics_entity_set_rotation(e2, e2_rotation);
}