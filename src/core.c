#include <GLFW/glfw3.h>
#include <light_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "menu.h"
#include "gjk.h"

#define GIM_ENTITY_COLOR (vec4) {1.0f, 1.0f, 1.0f, 1.0f}

static Entity e;

static void menu_dummy_callback()
{
	printf("dummy callback called!\n");
}

int core_init()
{
	Vertex* vertices = array_new(Vertex);
	u32* indices = array_new(indices);
	Vertex v;
	v.position = (vec2){4.0f, 11.0f};
	array_push(vertices, v);
	v.position = (vec2){4.0f, 5.0f};
	array_push(vertices, v);
	v.position = (vec2){9.0f, 9.0f};
	array_push(vertices, v);
	array_push(indices, 0);
	array_push(indices, 1);
	array_push(indices, 2);
	Mesh m1 = graphics_mesh_create(vertices, indices);
	graphics_entity_create_with_color(&e, m1, (vec2){0.0f, 0.0f}, (vec3){0.0f, 0.0f, 1.0f});

	vertices = array_new(Vertex);
	indices = array_new(indices);
	v;
	v.position = (vec2){5.0f, 7.0f};
	array_push(vertices, v);
	v.position = (vec2){7.0f, 3.0f};
	array_push(vertices, v);
	v.position = (vec2){10.0f, 2.0f};
	array_push(vertices, v);
	v.position = (vec2){12.0f, 7.0f};
	array_push(vertices, v);
	array_push(indices, 0);
	array_push(indices, 1);
	array_push(indices, 2);
	Mesh m2 = graphics_mesh_create(vertices, indices);
	graphics_entity_create_with_color(&e, m2, (vec2){0.0f, 0.0f}, (vec3){0.0f, 0.0f, 1.0f});

	printf("collision: %d\n", gjk(m1, m2));

	menu_register_dummy_callback(menu_dummy_callback);

	return 0;
}

void core_destroy()
{
}

void core_update(r32 delta_time)
{

}

void core_render()
{
	graphics_entity_render_basic_shader(&e);
}

void core_input_process(boolean* key_state, r32 delta_time)
{
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