#ifndef GIMMESH_MENU_H
#define GIMMESH_MENU_H
#include <GLFW/glfw3.h>
#include "common.h"

typedef void (*Dummy_Callback)();

void menu_register_dummy_callback(Dummy_Callback f);
void menu_char_click_process(GLFWwindow* window, u32 c);
void menu_key_click_process(GLFWwindow* window, s32 key, s32 scan_code, s32 action, s32 mods);
void menu_mouse_click_process(GLFWwindow* window, s32 button, s32 action, s32 mods);
void menu_scroll_change_process(GLFWwindow* window, s64 x_offset, s64 y_offset);
void menu_init(GLFWwindow* window);
void menu_render();
void menu_destroy();

#endif