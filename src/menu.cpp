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

typedef struct Joint_Definition {
	vec3 translation;
	vec3 rotation;
	vec3 scale;
	vec3 color;
	struct Joint_Definition* children;
} Joint_Definition;

static Joint_Definition root;

typedef void (*Hierarchical_Model_Set_Callback)(const Joint_Definition* joint_definition);

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

static Joint_Definition create_joint_definition()
{
	Joint_Definition jd;
	jd.children = array_create(Joint_Definition, 1);
	jd.translation = (vec3){0.0f, 0.0f, 0.0f};
	jd.rotation = (vec3){0.0f, 0.0f, 0.0f};
	jd.scale = (vec3){1.0f, 1.0f, 1.0f};
	jd.color = (vec3){1.0f, 0.0f, 0.0f};
	return jd;
}

static void destroy_joint_definition(Joint_Definition* jd)
{
	for (u32 i = 0; i < array_get_length(jd->children); ++i)
		destroy_joint_definition(&jd->children[i]);

	array_release(jd->children);
}

static void set_up_arm_pose()
{
	destroy_joint_definition(&root);
	root = create_joint_definition();
	root.translation = (vec3){-1.0f, 0.0f, 0.0f};
	root.scale = (vec3){0.61f, 0.025f, 0.025};
	root.color = (vec3){1.0f, 0.0f, 0.0f};
	
	Joint_Definition forearm = create_joint_definition();
	forearm.translation = (vec3){1.3f, 0.0f, 0.0f};
	forearm.scale = (vec3){0.61f, 0.025f, 0.025};
	forearm.color = (vec3){0.0f, 1.0f, 0.0f};

	Joint_Definition hand = create_joint_definition();
	hand.translation = (vec3){1.3f, 0.0f, 0.0f};
	hand.scale = (vec3){0.35f, 0.025f, 0.025};
	hand.color = (vec3){0.0f, 0.0f, 1.0f};

	array_push(forearm.children, &hand);
	array_push(root.children, &forearm);
}

static void display_joint_definition(Joint_Definition* joint)
{
	ImGui::PushID(joint);
	if (ImGui::DragFloat3("Translation", (r32*)&joint->translation, 0.01f, -FLT_MAX, FLT_MAX, "%.3f"))
	{
		hierarchical_model_set_callback(&root);
	}
	if (ImGui::DragFloat3("Rotation", (r32*)&joint->rotation, 0.1f, -180.0f, 180.0f, "%.3f"))
	{
		hierarchical_model_set_callback(&root);
	}
	if (ImGui::DragFloat3("Scale", (r32*)&joint->scale, 0.01f, 0.0f, FLT_MAX, "%.3f"))
	{
		hierarchical_model_set_callback(&root);
	}
	if (ImGui::DragFloat3("Color", (r32*)&joint->color, 0.01f, 0.0f, 1.0f, "%.3f"))
	{
		hierarchical_model_set_callback(&root);
	}

	const r32 INDENTATION = 50.0f;
	ImGui::Indent(INDENTATION);
	for (u32 i = 0; i < array_get_length(joint->children); ++i)
	{
		ImGui::Text("Joint %u", i);
		display_joint_definition(&joint->children[i]);
	}
	if (ImGui::Button("+"))
	{
		Joint_Definition jd = create_joint_definition();
		array_push(joint->children, &jd);
	}
	ImGui::SameLine(0.0f, -1.0f);
	if (ImGui::Button("-"))
	{
		array_pop(joint->children);
	}
	ImGui::Indent(-INDENTATION);

	ImGui::PopID();
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
		char buffer[64];

		ImGui::Text("Root Joint");
		display_joint_definition(&root);
	}

	if (ImGui::Button("Set up arm pose"))
	{
		set_up_arm_pose();
		hierarchical_model_set_callback(&root);
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

	root = create_joint_definition();
}

extern "C" void menu_destroy()
{
	// Cleanup
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();
}