#pragma once
#include <glad/gl.h>
#include <GLFW/glfw3.h>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/glm.hpp>
#include <glm/gtc/matrix_access.hpp>
#include <glm/gtx/vec_swizzle.hpp>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>

#include <iostream>
#include <format>

class Application
{
public:
    Application(uint32_t width, uint32_t height);

    ~Application() {}

    // GLFW, GLAD, ImGui init
    void init();

    // Start the app
    void run();

    // Example window
    void example_window();

    // Your application logic here
    void update();

    // GLFW, GLAD, ImGui cleanup
    void cleanup();

    GLFWwindow* window;
    uint32_t width;
    uint32_t height;
    bool is_running;

    // Your application variables here:
    uint32_t counter;
};