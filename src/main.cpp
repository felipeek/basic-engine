#define STB_IMAGE_IMPLEMENTATION
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define GRAPHICS_MATH_IMPLEMENT
#define GLSL_VERSION "#version 330"
#include <light_array.h>
#include <stb_image.h>
#include <stb_image_write.h>

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <stdio.h>
#include "core.h"
#include "vendor/imgui.h"
#include "vendor/imgui_impl_glfw.h"
#include "vendor/imgui_impl_opengl3.h"

#define WINDOW_TITLE "basic-engine"
#define DEFAULT_WINDOW_WIDTH 1366
#define DEFAULT_WINDOW_HEIGHT 768

// For simplicitly, the window size is updated in the main loop and globally available for all units
dvec2 window_size;
dvec2 framebuffer_size;

static Core_Ctx core_ctx;

static void glfw_key_callback(GLFWwindow* window, s32 key, s32 scanCode, s32 action, s32 mods)
{
	ImGuiIO& io = ImGui::GetIO();
	if (!io.WantCaptureKeyboard)
		core_key_press_process(&core_ctx, key, scanCode, action, mods);
}

static void glfw_cursor_callback(GLFWwindow* window, r64 x_pos, r64 y_pos)
{
	ImGuiIO& io = ImGui::GetIO();
	if (!io.WantCaptureMouse)
		core_mouse_change_process(&core_ctx, x_pos, y_pos);
}

static void glfw_mouse_button_callback(GLFWwindow* window, s32 button, s32 action, s32 mods)
{
	ImGuiIO& io = ImGui::GetIO();
	if (!io.WantCaptureMouse)
	{
		r64 x_pos, y_pos;
		glfwGetCursorPos(window, &x_pos, &y_pos);
		core_mouse_click_process(&core_ctx, button, action, x_pos, y_pos);
	}
}

static void glfw_scroll_callback(GLFWwindow* window, r64 x_offset, r64 y_offset)
{
	ImGuiIO& io = ImGui::GetIO();
	if (!io.WantCaptureMouse)
		core_scroll_change_process(&core_ctx, x_offset, y_offset);
}

static void glfw_window_resize_callback(GLFWwindow* window, s32 width, s32 height)
{
	window_size = (dvec2){width, height};
	core_window_resize_process(&core_ctx, width, height);
}

static void glfw_framebuffer_resize_callback(GLFWwindow* window, s32 width, s32 height)
{
	framebuffer_size = (dvec2){width, height};
}

static void glfw_char_callback(GLFWwindow* window, u32 c)
{
	ImGuiIO& io = ImGui::GetIO();
	if (!io.WantCaptureKeyboard) {}
}

static GLFWwindow* init_glfw()
{
	glfwInit();
	glfwWindowHint(GLFW_RESIZABLE, GLFW_TRUE);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GLFW_TRUE);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	GLFWwindow* window = glfwCreateWindow(DEFAULT_WINDOW_WIDTH, DEFAULT_WINDOW_HEIGHT, WINDOW_TITLE, 0, 0);
	glfwSetWindowPos(window, 50, 50);
	glfwMakeContextCurrent(window);
	glfwSetKeyCallback(window, glfw_key_callback);
	glfwSetCursorPosCallback(window, glfw_cursor_callback);
	glfwSetMouseButtonCallback(window, glfw_mouse_button_callback);
	glfwSetScrollCallback(window, glfw_scroll_callback);
	glfwSetWindowSizeCallback(window, glfw_window_resize_callback);
	glfwSetFramebufferSizeCallback(window, glfw_framebuffer_resize_callback);
	glfwSetCharCallback(window, glfw_char_callback);
	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_NORMAL);

	return window;
}

static ImGuiContext* init_imgui(GLFWwindow* window)
{
	// Setup Dear ImGui binding
	IMGUI_CHECKVERSION();
	ImGuiContext* imgui_ctx = ImGui::CreateContext();

	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init(GLSL_VERSION);

	// Setup style
	//ImGui::StyleColorsDark();
	ImGui::StyleColorsLight();

	return imgui_ctx;
}

static void imgui_destroy(ImGuiContext* imgui_ctx)
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext(imgui_ctx);
}

static void init_glew()
{
	glewExperimental = true;
	glewInit();
}

s32 main(s32 argc, s8** argv)
{
	r32 delta_time = 0.0f;

	GLFWwindow* main_window = init_glfw();
	init_glew();

	// Retrieve effective window size and framebuffer size
	s32 w, h;
	glfwGetWindowSize(main_window, &w, &h);
	window_size = (dvec2){w, h};
	glfwGetFramebufferSize(main_window, &w, &h);
	framebuffer_size = (dvec2){w, h};

	ImGuiContext* imgui_ctx = init_imgui(main_window);
	core_ctx = core_init(main_window);

	glEnable(GL_DEPTH_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

	r64 last_frame = glfwGetTime();
	s32 frame_number = (s32)last_frame;
	u32 fps = 0;

	while (!glfwWindowShouldClose(main_window))
	{
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
		glClearColor(0.2074f, 0.3168f, 0.3615f, 1.0f);

		core_input_process(&core_ctx, delta_time);
		core_update(&core_ctx, delta_time);
		core_render(&core_ctx);

		glfwPollEvents();
		glfwSwapBuffers(main_window);

		r64 current_frame = glfwGetTime();
		if ((s32)current_frame > frame_number)
		{
			//printf("FPS: %u\n", fps);
			fps = 0;
			frame_number++;
		}
		else
			++fps;

		delta_time = (r32)(current_frame - last_frame);

		last_frame = current_frame;
	}

	core_destroy(&core_ctx);
	imgui_destroy(imgui_ctx);
	glfwTerminate();
}
