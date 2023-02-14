#ifndef BASIC_ENGINE_CORE_H
#define BASIC_ENGINE_CORE_H

#include "common.h"
#include "graphics.h"
#include "ui.h"
#include <GLFW/glfw3.h>

typedef struct {
	GLFWwindow* window;

	Camera camera;
	Light* lights;
	Entity e;
	Entity e2;

	// Relevant for lookat camera
	bool is_rotating_camera;
	bool is_panning_camera;
	bool alternative_panning_method;

	bool key_state[1024];	// @TODO: Check range.
	bool is_ui_active;
	Ui_Ctx ui_ctx;
} Core_Ctx;

Core_Ctx core_init(GLFWwindow* window);
void core_destroy(Core_Ctx* ctx);
void core_update(Core_Ctx* ctx, r32 delta_time);
void core_render(Core_Ctx* ctx);
void core_input_process(Core_Ctx* ctx, r32 delta_time);
void core_key_press_process(Core_Ctx* ctx, s32 key, s32 scanCode, s32 action, s32 mods);
void core_mouse_change_process(Core_Ctx* ctx, r64 x_pos, r64 y_pos);
void core_mouse_click_process(Core_Ctx* ctx, s32 button, s32 action, r64 x_pos, r64 y_pos);
void core_scroll_change_process(Core_Ctx* ctx, r64 x_offset, r64 y_offset);
void core_window_resize_process(Core_Ctx* ctx, s32 width, s32 height);

#endif