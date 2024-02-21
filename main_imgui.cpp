// Dear ImGui: standalone example application for GLFW + OpenGL 3, using programmable pipeline
// (GLFW is a cross-platform general purpose library for handling windows, inputs, OpenGL/Vulkan/Metal graphics context creation, etc.)

// Learn about Dear ImGui:
// - FAQ                  https://dearimgui.com/faq
// - Getting Started      https://dearimgui.com/getting-started
// - Documentation        https://dearimgui.com/docs (same as your local docs/ folder).
// - Introduction, links and more at the top of imgui.cpp

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"
#include <stdio.h>
#include <iostream>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

#include "Ingris.hpp"

static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

void plotWorldState(const char *title, const Ingris& ingris) {
    auto worldState = ingris.simWorldState;
    auto config = ingris.config;
    double noseRadius = 0.1 * config.robotRadius;

    // Get the puck x- and y-coordinates as required for implot
    int np = worldState->pucks.size();
    double* puckXs = new double[np];
    double* puckYs = new double[np];
    for (int i = 0; i < np; ++i) {
        puckXs[i] = worldState->pucks[i].x;
        puckYs[i] = worldState->pucks[i].y;
    }

    // As above, but for robots.  Also get coordinates for the "noses" of the
    // robots, which are the points at the front of the robot, and are used to
    // draw the direction of the robot.
    int nr = worldState->robots.size();
    double* robotXs = new double[nr];
    double* robotYs = new double[nr];
    double* robotNoseXs = new double[nr];
    double* robotNoseYs = new double[nr];
    for (int i = 0; i < nr; ++i) {
        double x = worldState->robots[i].x;
        double y = worldState->robots[i].y;
        double theta = worldState->robots[i].theta;
        robotXs[i] = x;
        robotYs[i] = y;
        robotNoseXs[i] = x + (config.robotRadius - noseRadius) * cos(theta);
        robotNoseYs[i] = y + (config.robotRadius - noseRadius) * sin(theta);
    }

    ImGui::Begin(title);
    ImPlot::SetNextAxesLimits(0, config.width, 0, config.height);
    if (ImPlot::BeginPlot(title, ImVec2(config.width,config.height), ImPlotFlags_::ImPlotFlags_Equal | ImPlotFlags_::ImPlotFlags_NoTitle)) {
        ImPlot::SetupLegend(ImPlotLocation_West, ImPlotLegendFlags_Outside);

        ImVec2 plotSizeInPixels = ImPlot::GetPlotSize();
        ImPlotRect plotRect = ImPlot::GetPlotLimits();
        double scaleFactor = plotSizeInPixels.x / (plotRect.X.Max - plotRect.X.Min);

        //std::cout << "plotSizeInPixels: " << plotSizeInPixels.x << ", " << plotSizeInPixels.y << "\n";
        //double scaleFactor = plotSizeInPixels.x / config.width;

        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, scaleFactor * config.puckRadius, ImPlot::GetColormapColor(0), IMPLOT_AUTO, ImPlot::GetColormapColor(0));
        ImPlot::PlotScatter("Pucks", puckXs, puckYs, np);
        ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);

        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, scaleFactor * config.robotRadius, ImPlot::GetColormapColor(1), IMPLOT_AUTO, ImPlot::GetColormapColor(1));
        ImPlot::PlotScatter("Robots", robotXs, robotYs, nr);

        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, scaleFactor * noseRadius, ImPlot::GetColormapColor(2), IMPLOT_AUTO, ImPlot::GetColormapColor(2));
        ImPlot::PlotScatter("Robot Orientations", robotNoseXs, robotNoseYs, nr);
        ImPlot::PopStyleVar();
        ImPlot::EndPlot();
    }
    ImGui::End();
}

// Main code
int main(int, char**)
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

    // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char* glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char* glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char* glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    //glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    //glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    // Create window with graphics context
    GLFWwindow* window = glfwCreateWindow(1280, 720, "ingris", nullptr, nullptr);
    if (window == nullptr)
        return 1;
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui and ImPlot context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;     // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;      // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    //ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    // Our state
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    Ingris ingris;

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        ingris.runSim();
        ingris.simWorldState->print();

        // Poll and handle events (inputs, window resize, etc.)
        // You can read the io.WantCaptureMouse, io.WantCaptureKeyboard flags to tell if dear imgui wants to use your inputs.
        // - When io.WantCaptureMouse is true, do not dispatch mouse input data to your main application, or clear/overwrite your copy of the mouse data.
        // - When io.WantCaptureKeyboard is true, do not dispatch keyboard input data to your main application, or clear/overwrite your copy of the keyboard data.
        // Generally you may always pass all inputs to dear imgui, and hide them from your application based on those two flags.
        glfwPollEvents();

        // Start the Dear ImGui frame
        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // 2. Show a simple window that we create ourselves. We use a Begin/End pair to create a named window.
        {
            static float f = 0.0f;
            static int counter = 0;

            ImGui::Begin("Hello, world!");                          // Create a window called "Hello, world!" and append into it.

            ImGui::Text("This is some useful text.");               // Display some text (you can use a format strings too)

            ImGui::SliderFloat("float", &f, 0.0f, 1.0f);            // Edit 1 float using a slider from 0.0f to 1.0f
            ImGui::ColorEdit3("clear color", (float*)&clear_color); // Edit 3 floats representing a color

            if (ImGui::Button("Button"))                            // Buttons return true when clicked (most widgets return true when edited/activated)
                counter++;
            ImGui::SameLine();
            ImGui::Text("counter = %d", counter);

            ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
            ImGui::End();
        }

        plotWorldState("Simulation", ingris);
        //plotWorldState("Prediction");

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w, clear_color.z * clear_color.w, clear_color.w);
        glClear(GL_COLOR_BUFFER_BIT);
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
    }

    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();

    glfwDestroyWindow(window);
    glfwTerminate();

    return 0;
}
