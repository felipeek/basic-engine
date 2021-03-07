#ifndef BASIC_ENGINE_COLLISION_H
#define BASIC_ENGINE_COLLISION_H

#include "common.h"
#include "graphics.h"

boolean collision_check_point_side_of_triangle(vec3 point, vec3 t1, vec3 t2, vec3 t3);
boolean collision_check_edge_collides_triangle(vec3 edge_p1, vec3 edge_p2, vec3 t1, vec3 t2, vec3 t3, r32* d, vec3* intersection);
boolean collision_check_dynamic_collision_between_point_and_entity_face(
	vec3 initial_point_position,
	vec3 final_point_position,
	vec3 initial_entity_position,
	vec3 final_entity_position,
	Quaternion initial_entity_rotation,
	Quaternion final_entity_rotation,
	vec3 entity_scale,
	vec3 face_point_local_coords_1,
	vec3 face_point_local_coords_2,
	vec3 face_point_local_coords_3);

#endif