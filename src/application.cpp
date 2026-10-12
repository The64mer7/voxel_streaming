#include "application.hpp"

Application::Application(uint32_t width, uint32_t height)
    : width(width), height(height), window(nullptr), is_running(true)
{
}

// GLFW, GLAD, ImGui init

void Application::init()
{
    if (!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW!\n";
        return;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    window = glfwCreateWindow(width, height, "Engine Viewport", nullptr, nullptr);
    if (!window)
    {
        std::cerr << "Failed to create GLFW window!\n";
        glfwTerminate();
        return;
    }
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    if (!gladLoadGL(glfwGetProcAddress))
    {
        printf("Failed to initialize GLAD\n");
        return;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;

    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
    io.ConfigFlags |= ImGuiConfigFlags_ViewportsEnable;

    ImGui::StyleColorsDark();

    ImGuiStyle& style = ImGui::GetStyle();
    if (io.ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
    {
        style.WindowRounding = 0.0f;
        style.Colors[ImGuiCol_WindowBg].w = 1.0f;
    }

    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init("#version 330");

    start();
}

// Start the app

void Application::run()
{
    float last_time = (float)glfwGetTime();
    while (is_running && !glfwWindowShouldClose(window))
    {
        float current_time = (float)glfwGetTime();
        float delta_time = current_time - last_time;
        last_time = current_time;

        glfwPollEvents();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        bool opt_fullscreen = true;
        bool opt_padding = false;
        ImGuiWindowFlags window_flags = ImGuiWindowFlags_MenuBar | ImGuiWindowFlags_NoDocking;

        if (opt_fullscreen)
        {
            const ImGuiViewport* viewport = ImGui::GetMainViewport();
            ImGui::SetNextWindowPos(viewport->WorkPos);
            ImGui::SetNextWindowSize(viewport->WorkSize);
            ImGui::SetNextWindowViewport(viewport->ID);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
            ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
            window_flags |= ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
                            ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove;
            window_flags |= ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus;
        }

        if (opt_padding)
            ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

        if (ImGui::Begin("Engine Dockspace Window", &is_running, window_flags))
        {

            if (opt_padding)
                ImGui::PopStyleVar();

            if (opt_fullscreen)
                ImGui::PopStyleVar(2);

            ImGuiIO& io = ImGui::GetIO();
            if (io.ConfigFlags & ImGuiConfigFlags_DockingEnable)
            {
                ImGuiID dockspace_id = ImGui::GetID("MyDockSpace");
                ImGui::DockSpace(dockspace_id, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_None);
            }

            if (ImGui::BeginMenuBar())
            {
                if (ImGui::BeginMenu("File"))
                {
                    if (ImGui::MenuItem("Exit", "Alt+F4"))
                    {
                        is_running = false;
                    }
                    ImGui::EndMenu();
                }
                ImGui::EndMenuBar();
            }
        }
        ImGui::End();

        update(delta_time);

        ImGui::Render();

        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT);

        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_ViewportsEnable)
        {
            GLFWwindow* backup_current_context = glfwGetCurrentContext();
            ImGui::UpdatePlatformWindows();
            ImGui::RenderPlatformWindowsDefault();
            glfwMakeContextCurrent(backup_current_context);
        }

        glfwSwapBuffers(window);
    }
}

// Your application logic here

void Application::update(float dt)
{
    tick(dt);
    render(dt);
}

// GLFW, GLAD, ImGui cleanup

void Application::cleanup()
{
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();
}
