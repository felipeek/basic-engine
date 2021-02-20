#ifndef BASIC_ENGINE_MENU_H
#define BASIC_ENGINE_MENU_H
#include <GLFW/glfw3.h>
#include "common.h"

typedef enum {
	ROTATION_AXIS_X,
	ROTATION_AXIS_Y,
	ROTATION_AXIS_Z
} Rotation_Axis;

typedef struct Joint_Definition {
	vec3 translation;
	vec3 rotation;
	vec3 scale;
	vec3 color;
	Rotation_Axis rotation_axis;
	boolean follow_target_point;
	vec3 target_point;
	struct Joint_Definition* children;
} Joint_Definition;

typedef enum {
	IK_METHOD_INVERSE,
	IK_METHOD_PSEUDO_INVERSE,
	IK_METHOD_TRANSPOSE
} IK_Method;

typedef void (*Hierarchical_Model_Set_Callback)(const Joint_Definition* joint_definition);
typedef void (*Animate_Callback)(IK_Method ik_method);
typedef void (*Stop_Callback)();

void menu_register_hierarchical_model_set_callback(Hierarchical_Model_Set_Callback f);
void menu_register_animate_callback(Animate_Callback f);
void menu_register_stop_callback(Stop_Callback f);
void menu_char_click_process(GLFWwindow* window, u32 c);
void menu_key_click_process(GLFWwindow* window, s32 key, s32 scan_code, s32 action, s32 mods);
void menu_mouse_click_process(GLFWwindow* window, s32 button, s32 action, s32 mods);
void menu_scroll_change_process(GLFWwindow* window, s64 x_offset, s64 y_offset);
void menu_init(GLFWwindow* window);
void menu_render();
void menu_destroy();

#endif