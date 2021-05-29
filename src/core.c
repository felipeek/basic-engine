#include <GLFW/glfw3.h>
#include <light_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "menu.h"
#include "gjk.h"

#define GIM_ENTITY_COLOR (vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Entity e1, e2, penetration_line;

static void menu_dummy_callback()
{
	printf("dummy callback called!\n");
}

static u32* generate_indices_for(Vertex* vertices) {
	u32* indices = array_new(u32);
	for (u32 i = 1; i < array_length(vertices); ++i) {
		array_push(indices, i - 1);
		array_push(indices, i);
	}

	array_push(indices, array_length(vertices) - 1);
	array_push(indices, 0);
	return indices;
}

static void add_vertex(Vertex** vertices, r32 x, r32 y) {
	Vertex v;
	v.position = (vec2){x, y};
	array_push(*vertices, v);
}

int core_init()
{
	Vertex* vertices;

	vertices = array_new(Vertex);
	add_vertex(&vertices, 0.1f, 0.1f);
	add_vertex(&vertices, 0.8f, 0.2f);
	add_vertex(&vertices, 0.85f, 0.6f);
	add_vertex(&vertices, 0.3f, 0.55f);
	Mesh m = graphics_mesh_create(vertices, generate_indices_for(vertices));
	graphics_entity_create_with_color(&e1, m, (vec2){0.0f, 0.0f}, (vec3){0.0f, 0.0f, 1.0f});	

	vertices = array_new(Vertex);
	add_vertex(&vertices, 0.12f, 0.1f);
	add_vertex(&vertices, 0.2f, 0.2f);
	add_vertex(&vertices, 0.3f, 0.2f);
	add_vertex(&vertices, 0.4f, 0.15f);
	m = graphics_mesh_create(vertices, generate_indices_for(vertices));
	graphics_entity_create_with_color(&e2, m, (vec2){0.0f, 0.0f}, (vec3){0.0f, 0.0f, 1.0f});	

	vertices = array_new(Vertex);
	add_vertex(&vertices, 0.0f, 0.0f);
	add_vertex(&vertices, 0.0f, 0.0f);
	m = graphics_mesh_create(vertices, generate_indices_for(vertices));
	graphics_entity_create_with_color(&penetration_line, m, (vec2){0.0f, 0.0f}, (vec3){1.0f, 1.0f, 1.0f});	

	menu_register_dummy_callback(menu_dummy_callback);

	return 0;
}

void core_destroy()
{
}

void core_update(r32 delta_time)
{
	Simplex s;
	if (gjk(e1.mesh, e2.mesh, &s)) {
		e1.color = (vec3){1.0f, 0.0f, 0.0f};
		e2.color = (vec3){1.0f, 0.0f, 0.0f};

		vec2 result = epa(s, e1.mesh, e2.mesh);
		penetration_line.mesh.vertices[1].position = result;
		graphics_mesh_update(penetration_line.mesh);
	} else {
		e1.color = (vec3){0.0f, 1.0f, 0.0f};
		e2.color = (vec3){0.0f, 1.0f, 0.0f};
		penetration_line.mesh.vertices[1].position = (vec2){0.0f, 0.0f};
		graphics_mesh_update(penetration_line.mesh);
	}
}

void core_render()
{
	graphics_entity_render_basic_shader(&e1);
	graphics_entity_render_basic_shader(&e2);
	graphics_entity_render_basic_shader(&penetration_line);
}

void core_input_process(boolean* key_state, r32 delta_time)
{
	if (key_state[GLFW_KEY_UP]) {
		for (u32 i = 0; i < array_length(e2.mesh.vertices); ++i) {
			e2.mesh.vertices[i].position.y += 0.01f;
		}
		graphics_mesh_update(e2.mesh);
	}
	if (key_state[GLFW_KEY_DOWN]) {
		for (u32 i = 0; i < array_length(e2.mesh.vertices); ++i) {
			e2.mesh.vertices[i].position.y -= 0.01f;
		}
		graphics_mesh_update(e2.mesh);
	}
	if (key_state[GLFW_KEY_RIGHT]) {
		for (u32 i = 0; i < array_length(e2.mesh.vertices); ++i) {
			e2.mesh.vertices[i].position.x += 0.01f;
		}
		graphics_mesh_update(e2.mesh);
	}
	if (key_state[GLFW_KEY_LEFT]) {
		for (u32 i = 0; i < array_length(e2.mesh.vertices); ++i) {
			e2.mesh.vertices[i].position.x -= 0.01f;
		}
		graphics_mesh_update(e2.mesh);
	}
}

void core_mouse_change_process(boolean reset, r64 x_pos, r64 y_pos)
{
}

void core_mouse_click_process(s32 button, s32 action, r64 x_pos, r64 y_pos)
{

}

void core_scroll_change_process(r64 x_offset, r64 y_offset)
{

}

void core_window_resize_process(s32 width, s32 height)
{
}