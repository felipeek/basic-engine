#define IMGUI_IMPL_OPENGL_LOADER_GLEW
#include "common.h"
#include "vendor/imgui.h"
#include "vendor/imgui_impl_glfw.h"
#include "vendor/imgui_impl_opengl3.h"
#include <stdio.h>

#include <dirent.h>
#include <light_array.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLSL_VERSION "#version 330"
#define MENU_TITLE "Basic Engine"

typedef void (*Dummy_Callback)();
typedef void (*Restitution_Callback)(r32);
typedef void (*Gravity_Callback)(r32);
typedef void (*Static_Friction_Coefficient_Callback)(r32);
typedef void (*Dynamic_Friction_Coefficient_Callback)(r32);

static Dummy_Callback dummy_callback;
static Restitution_Callback restitution_callback;
static Gravity_Callback gravity_callback;
static Static_Friction_Coefficient_Callback static_friction_coefficient_callback;
static Dynamic_Friction_Coefficient_Callback dynamic_friction_coefficient_callback;

extern "C" void menu_register_dummy_callback(Dummy_Callback f)
{
	dummy_callback = f;
}

extern "C" void menu_register_restitution_callback(Restitution_Callback f)
{
	restitution_callback = f;
}

extern "C" void menu_register_gravity_callback(Gravity_Callback f)
{
	gravity_callback = f;
}

extern "C" void menu_register_static_friction_coefficient_callback(Gravity_Callback f)
{
	static_friction_coefficient_callback = f;
}

extern "C" void menu_register_dynamic_friction_coefficient_callback(Gravity_Callback f)
{
	dynamic_friction_coefficient_callback = f;
}

extern "C" void menu_char_click_process(GLFWwindow* window, u32 c)
{
	ImGui_ImplGlfw_CharCallback(window, c);
}

extern "C" void menu_key_click_process(GLFWwindow* window, s32 key, s32 scan_code, s32 action, s32 mods)
{
	ImGui_ImplGlfw_KeyCallback(window, key, scan_code, action, mods);
}

extern "C" void menu_mouse_click_process(GLFWwindow* window, s32 button, s32 action, s32 mods)
{
	ImGui_ImplGlfw_MouseButtonCallback(window, button, action, mods);
}

extern "C" void menu_scroll_change_process(GLFWwindow* window, s64 x_offset, s64 y_offset)
{
	ImGui_ImplGlfw_ScrollCallback(window, x_offset, y_offset);
}

extern "C" void menu_init(GLFWwindow* window)
{
	// Setup Dear ImGui binding
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGuiIO& io = ImGui::GetIO(); (void)io;

	ImGui_ImplGlfw_InitForOpenGL(window, false);
	ImGui_ImplOpenGL3_Init(GLSL_VERSION);

	// Setup style
	ImGui::StyleColorsDark();
}

static void draw_main_window()
{
	// Main body of the Demo window starts here.
	if (!ImGui::Begin(MENU_TITLE, 0, 0))
	{
		// Early out if the window is collapsed, as an optimization.
		ImGui::End();
		return;
	}

	static r32 restitution = 0.1f;
	if (ImGui::DragFloat("Restitution", &restitution, 0.01f, 0.0f, 1.0f))
	{
		if (restitution_callback)
		{
			restitution_callback(restitution);
		}
	}

	static r32 gravity = 10.0f;
	if (ImGui::DragFloat("Gravity", &gravity, 1.0f, 0.0f, 100.0f))
	{
		if (gravity_callback)
		{
			gravity_callback(gravity);
		}
	}

	static r32 static_friction_coefficient = 1.0f;
	if (ImGui::DragFloat("Static Friction Coefficient", &static_friction_coefficient, 0.01f, 0.0f, 1.0f))
	{
		if (static_friction_coefficient_callback)
		{
			static_friction_coefficient_callback(static_friction_coefficient);
		}
	}

	static r32 dynamic_friction_coefficient = 1.0f;
	if (ImGui::DragFloat("Dynamic Friction Coefficient", &dynamic_friction_coefficient, 0.01f, 0.0f, 1.0f))
	{
		if (dynamic_friction_coefficient_callback)
		{
			dynamic_friction_coefficient_callback(dynamic_friction_coefficient);
		}
	}

	ImGui::End();
}

extern "C" void menu_render()
{
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	
	ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(550, 680), ImGuiCond_FirstUseEver);

	draw_main_window();

	// Rendering
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

extern "C" void menu_destroy()
{
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}