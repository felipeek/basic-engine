#ifndef BASIC_ENGINE_COLLISION_H
#define BASIC_ENGINE_COLLISION_H

#include "common.h"
#include "graphics.h"

s32 collision_check_point_side_of_triangle(vec3 point, vec3 t1, vec3 t2, vec3 t3);
s32 collision_check_edge_collides_triangle(vec3 edge_p1, vec3 edge_p2, vec3 t1, vec3 t2, vec3 t3, r32* d, vec3* intersection);
s32 collision_vertex_face_test(
	vec4 old_frame_point_position,
	vec4 old_frame_face_position,
	Quaternion old_frame_face_rotation,
	vec4 new_frame_point_position,
	vec4 new_frame_face_position,
	Quaternion new_frame_face_rotation,
	vec3 face_scale);
#endif