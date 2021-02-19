#define IMGUI_IMPL_OPENGL_LOADER_GLEW
#include "common.h"
#include "vendor/imgui.h"
#include "vendor/imgui_impl_glfw.h"
#include "vendor/imgui_impl_opengl3.h"
#include <stdio.h>

#include <dirent.h>
#include <dynamic_array.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>

#define GLSL_VERSION "#version 330"
#define MENU_TITLE "Basic Engine"
#define MAX_NUM_JOINTS 64

typedef struct {
	float x, y, z;
} vec3;

typedef void (*Hierarchical_Model_Set_Callback)(u32 num_joints, vec3* translations, vec3* rotations, vec3* scales);

static Hierarchical_Model_Set_Callback hierarchical_model_set_callback;

extern "C" void menu_register_hierarchical_model_set_callback(Hierarchical_Model_Set_Callback f)
{
	hierarchical_model_set_callback = f;
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

	if (ImGui::CollapsingHeader("Hierarchical Model Creator", 0))
	{
		static u32 number_of_joints = 1;
		static vec3 translations[MAX_NUM_JOINTS];
		static vec3 rotations[MAX_NUM_JOINTS];
		static vec3 scales[MAX_NUM_JOINTS];
		char buffer[64];

		if (ImGui::Button("+"))
		{
			if (number_of_joints < MAX_NUM_JOINTS)
				++number_of_joints;
		}

		ImGui::SameLine(0.0f, -1.0f);
		if (ImGui::Button("-"))
		{
			if (number_of_joints > 1)
				--number_of_joints;
		}

		for (u32 i = 0; i < number_of_joints; ++i) {
			ImGui::Text("Joint %u", i);
			if (ImGui::DragFloat3("Translation", (r32*)&translations[i], 0.1f, -FLT_MAX, FLT_MAX, "%.3f"))
			{
				hierarchical_model_set_callback(number_of_joints, translations, rotations, scales);
			}
			if (ImGui::DragFloat3("Rotation", (r32*)&rotations[i], 0.1f, -FLT_MAX, FLT_MAX, "%.3f"))
			{
				hierarchical_model_set_callback(number_of_joints, translations, rotations, scales);
			}
			if (ImGui::DragFloat3("Scale", (r32*)&scales[i], 0.1f, -FLT_MAX, FLT_MAX, "%.3f"))
			{
				hierarchical_model_set_callback(number_of_joints, translations, rotations, scales);
			}
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