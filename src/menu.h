#ifndef GIMMESH_MENU_H
#define GIMMESH_MENU_H
#include <GLFW/glfw3.h>
#include "common.h"

typedef void (*Dummy_Callback)();

void menu_register_dummy_callback(Dummy_Callback f);
void menu_init(GLFWwindow* window);
void menu_render();
void menu_destroy();

#endif