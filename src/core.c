#include <GLFW/glfw3.h>
#include <light_array.h>
#include <stdio.h>
#include <math.h>
#include "core.h"
#include "graphics.h"
#include "menu.h"

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
	v.position = (vec2){-0.5f, -0.5f};
	array_push(vertices, v);
	v.position = (vec2){0.5f, -0.5f};
	array_push(vertices, v);
	v.position = (vec2){0.0f, 0.5f};
	array_push(vertices, v);
	array_push(indices, 0);
	array_push(indices, 1);
	array_push(indices, 2);
	Mesh m = graphics_mesh_create(vertices, indices);
	graphics_entity_create_with_color(&e, m, (vec2){0.0f, 0.0f}, (vec3){0.0f, 0.0f, 1.0f});

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