#define IMGUI_IMPL_OPENGL_LOADER_GLEW
#include "ui.h"
#include <stdio.h>
#include "vendor/imgui.h"
#include "vendor/imgui.h"
#include "vendor/imgui_impl_glfw.h"
#include "vendor/imgui_impl_opengl3.h"

#define MENU_TITLE "Menu (press [ESC] to show/hide)"

Ui_Ctx ui_init()
{
	Ui_Ctx ctx;
	return ctx;
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

	if (ImGui::Button("Dummy"))
	{
		printf("Hello World!\n");
	}

	ImGui::End();
}

void ui_render(Ui_Ctx* ctx, bool is_active)
{
	// Start the Dear ImGui frame
	ImGui_ImplOpenGL3_NewFrame();
	ImGui_ImplGlfw_NewFrame();
	ImGui::NewFrame();
	
	ImGui::SetNextWindowPos(ImVec2(650, 20), ImGuiCond_FirstUseEver);
	ImGui::SetNextWindowSize(ImVec2(550, 680), ImGuiCond_FirstUseEver);

	if (is_active)
		draw_main_window();

	// Rendering
	ImGui::Render();
	ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
}

void ui_destroy(Ui_Ctx* ctx)
{
}