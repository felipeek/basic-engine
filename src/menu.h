#ifndef BASIC_ENGINE_MENU_H
#define BASIC_ENGINE_MENU_H
#include <GLFW/glfw3.h>
#include "common.h"

typedef void (*Hierarchical_Model_Set_Callback)(u32 num_joints, vec3* translations, vec3* rotations, vec3* scales);

void menu_register_hierarchical_model_set_callback(Hierarchical_Model_Set_Callback f);
void menu_char_click_process(GLFWwindow* window, u32 c);
void menu_key_click_process(GLFWwindow* window, s32 key, s32 scan_code, s32 action, s32 mods);
void menu_mouse_click_process(GLFWwindow* window, s32 button, s32 action, s32 mods);
void menu_scroll_change_process(GLFWwindow* window, s64 x_offset, s64 y_offset);
void menu_init(GLFWwindow* window);
void menu_render();
void menu_destroy();

#endif