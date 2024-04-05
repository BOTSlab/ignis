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
#include <iostream>
#include <sstream>
#include <stdio.h>
#define GL_SILENCE_DEPRECATION
#if defined(IMGUI_IMPL_OPENGL_ES2)
#include <GLES2/gl2.h>
#endif
#include <GLFW/glfw3.h> // Will drag system OpenGL headers

#include "DarsScenario.hpp"
#include "CommonTypes.hpp"

const ImVec4 red = ImVec4(1.0f, 0.0f, 0.0f, 1.00f);
const ImVec4 green = ImVec4(0.0f, 1.0f, 0.0f, 1.00f);
const ImVec4 blue = ImVec4(0.0f, 0.0f, 1.0f, 1.00f);
const ImVec4 gray = ImVec4(0.7f, 0.7f, 0.7f, 1.00f);
const ImVec4 clear = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);

// Global variables.
int selectedRobotIndex = -1;

static void glfw_error_callback(int error, const char *description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

void plotInteraction(double scaleFactor, WorldConfig &config, std::__1::shared_ptr<WorldState> &worldState)
{
    if (ImPlot::IsPlotHovered() && ImGui::IsMouseReleased(ImGuiMouseButton_Left))
    {
        ImVec2 mousePos = ImGui::GetMousePos();
        ImPlotPoint plotMousePos = ImPlot::GetPlotMousePos();
        double mouseX = plotMousePos.x;
        double mouseY = plotMousePos.y;

        // Check if the mouse click is within the robot scatter plot
        selectedRobotIndex = -1;
        for (int i = 0; i < worldState->robots.size(); ++i)
        {
            double robotX = worldState->robots[i].x;
            double robotY = worldState->robots[i].y;
            double distance = sqrt((mouseX - robotX) * (mouseX - robotX) + (mouseY - robotY) * (mouseY - robotY));

            // If the mouse click is within the robot, set the selectedRobotIndex to the current robot index
            if (distance <= config.robotRadius)
            {
                selectedRobotIndex = i;
                break;
            }
        }
    }

    // Handle arrow keys to control the selected robot's speed
    if (selectedRobotIndex != -1)
    {
        ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 1.1 * scaleFactor * config.robotRadius, ImPlot::GetColormapColor(1), IMPLOT_AUTO, ImPlot::GetColormapColor(1));
        double robotX = worldState->robots[selectedRobotIndex].x;
        double robotY = worldState->robots[selectedRobotIndex].y;
        ImPlot::PlotScatter("Selected Robot", &robotX, &robotY, 1);

        // The selected robot will be manually controlled, with zero speed by default.
        worldState->robots[selectedRobotIndex].controlInput.forwardSpeed = 0;
        worldState->robots[selectedRobotIndex].controlInput.angularSpeed = 0;

        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_UpArrow)))
        {
            worldState->robots[selectedRobotIndex].controlInput.forwardSpeed = 1.5;
        }
        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_DownArrow)))
        {
            worldState->robots[selectedRobotIndex].controlInput.forwardSpeed = -1.5;
        }
        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_LeftArrow)))
        {
            worldState->robots[selectedRobotIndex].controlInput.angularSpeed = 0.1;
        }
        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_RightArrow)))
        {
            worldState->robots[selectedRobotIndex].controlInput.angularSpeed = -0.1;
        }
    }
}

void perRobotPlots(size_t robotIndex, const DarsScenario &darsScenario, double scaleFactor)
{
//cout << "perRobotPlots - START" << endl;
    auto worldState = darsScenario.simWorldState;
    auto config = darsScenario.config;
    double noseRadius = 0.1 * config.robotRadius;
    auto color = ImPlot::GetColormapColor(robotIndex + 2);

    auto &robot = worldState->robots[robotIndex];

    ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, scaleFactor * config.robotRadius, color, IMPLOT_AUTO, color);
    std::ostringstream oss;
    oss << "Robot " << robotIndex;
    std::string robotString = oss.str();
    ImPlot::PlotScatter(robotString.c_str(), &robot.x, &robot.y, 1);

    double noseX = robot.x + (config.robotRadius - noseRadius) * cos(robot.theta);
    double noseY = robot.y + (config.robotRadius - noseRadius) * sin(robot.theta);
    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, scaleFactor * noseRadius, color, IMPLOT_AUTO, color);
    ImPlot::PlotScatter(robotString.c_str(), &noseX, &noseY, 1);
    ImPlot::PopStyleVar();

    ImPlot::SetNextLineStyle(color, scaleFactor * config.curveThickness);
    if (darsScenario.robotIndexToCurveMap.count(robotIndex) > 0) {
        std::ostringstream oss;
        oss << "Curve for robot " << robotIndex;
        const Curve &curve = darsScenario.robotIndexToCurveMap.at(robotIndex);
        std::vector<double> xVals, yVals;
        for (int i=0; i<curve.points.size() - 1; i++) {
            const CurvePoint &p1 = curve.points[i];
            const CurvePoint &p2 = curve.points[i+1];
            xVals.push_back(p1.pose.x);
            yVals.push_back(p1.pose.y);
            xVals.push_back(p2.pose.x);
            yVals.push_back(p2.pose.y);
        }
        ImPlot::PlotLine(oss.str().c_str(), xVals.data(), yVals.data(), xVals.size());
    }

    if (darsScenario.robotIndexToSensorReadingsMap.count(robotIndex) > 0) {
        std::ostringstream oss;
        oss << "Sensors for robot " << robotIndex;
        double sensorSize = 0.1 * config.robotRadius;

        for (const SensorReading &sensorReading : darsScenario.robotIndexToSensorReadingsMap.at(robotIndex)) {
            const SensorPosition &sensorPos = sensorReading.position;
            Vec2 pos = Vec2(robot.x, robot.y) + Vec2(sensorPos.forwardOffset, sensorPos.lateralOffset).rotate(robot.theta);
            if (sensorReading.value == 0) {
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, scaleFactor * sensorSize, gray, IMPLOT_AUTO, gray);
            } else {
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, scaleFactor * sensorSize, color, IMPLOT_AUTO, color);
            }
            ImPlot::PlotScatter(oss.str().c_str(), &pos.x, &pos.y, 1);
        }
    }
//cout << "perRobotPlots - END" << endl;
}

void plotWorldState(const char *title, const DarsScenario &darsScenario)
{
    auto worldState = darsScenario.simWorldState;
    auto config = darsScenario.config;
    double noseRadius = 0.1 * config.robotRadius;

    double boundaryXs[] = {0, static_cast<double>(config.width), static_cast<double>(config.width), 0};
    double boundaryYs[] = {0, 0, static_cast<double>(config.height), static_cast<double>(config.height)};

    ImGui::Begin(title);
    double buffer = 100;
    double plotScale = 1.55;
    ImPlot::SetNextAxesLimits(-buffer, config.width + buffer, -buffer, config.height + buffer);
    if (ImPlot::BeginPlot(title, ImVec2(plotScale*config.width, plotScale*config.height), ImPlotFlags_::ImPlotFlags_Equal | ImPlotFlags_::ImPlotFlags_NoTitle))
    {
        ImPlot::SetupLegend(ImPlotLocation_East, ImPlotLegendFlags_Outside);

        // Draw the boundaries of the world.
        ImPlot::PlotLine("Boundary", boundaryXs, boundaryYs, 4, ImPlotLineFlags_Loop, ImPlotLineFlags_Shaded);

        // Setup the scaling factor for the markers.
        ImVec2 plotSizeInPixels = ImPlot::GetPlotSize();
        ImPlotRect plotRect = ImPlot::GetPlotLimits();
        double scaleFactor = plotSizeInPixels.x / (plotRect.X.Max - plotRect.X.Min);

        std::vector<double> puckXs, puckYs;
        for (int i = 0; i < worldState->pucks.size(); ++i)
        {
            puckXs.push_back(worldState->pucks[i].x);
            puckYs.push_back(worldState->pucks[i].y);
        }
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, scaleFactor * config.puckRadius, ImPlot::GetColormapColor(1), IMPLOT_AUTO, ImPlot::GetColormapColor(1));
        ImPlot::PlotScatter("Pucks", puckXs.data(), puckYs.data(), puckXs.size());

        for (int i = 0; i < worldState->robots.size(); ++i)
            perRobotPlots(i, darsScenario, scaleFactor);

        plotInteraction(scaleFactor, config, worldState);

        ImPlot::EndPlot();
    }
    ImGui::End();
}

void handleControlsWindow(DarsScenario &darsScenario, ImGuiIO &io)
{
    static float forwardSpeed = 0.0;
    static float angularSpeed = 0.0;

    ImGui::Begin("Controls");

    if (ImGui::Button("Reset"))
        darsScenario.prepareToReset();

    if (!darsScenario.isPaused() && ImGui::Button("Pause"))
        darsScenario.prepareToPause();

    if (darsScenario.isPaused() && ImGui::Button("Play"))
        darsScenario.prepareToUnpause();

    if (darsScenario.isPaused() && ImGui::Button("Step"))
        darsScenario.prepareToStepOnce();

    // ImGui::SameLine();

    ImGui::Text("step count: %d", darsScenario.getStepCount());
    ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    ImGui::End();
}

// Main code
int main(int, char **)
{
    glfwSetErrorCallback(glfw_error_callback);
    if (!glfwInit())
        return 1;

        // Decide GL+GLSL versions
#if defined(IMGUI_IMPL_OPENGL_ES2)
    // GL ES 2.0 + GLSL 100
    const char *glsl_version = "#version 100";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 2);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_CLIENT_API, GLFW_OPENGL_ES_API);
#elif defined(__APPLE__)
    // GL 3.2 + GLSL 150
    const char *glsl_version = "#version 150";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 2);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE); // 3.2+ only
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);           // Required on Mac
#else
    // GL 3.0 + GLSL 130
    const char *glsl_version = "#version 130";
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    // glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);  // 3.2+ only
    // glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);            // 3.0+ only
#endif

    // Create window with graphics context
    GLFWwindow *window = glfwCreateWindow(2000, 850, "DarsScenario", nullptr, nullptr);
    if (window == nullptr)
        return 1;
    glfwSetWindowPos(window, 50, 0);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1); // Enable vsync

    // Setup Dear ImGui and ImPlot context
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO &io = ImGui::GetIO();
    (void)io;
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableGamepad;  // Enable Gamepad Controls

    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    DarsScenario darsScenario;

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        darsScenario.step();

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

        handleControlsWindow(darsScenario, io);

        plotWorldState("Simulation", darsScenario);
        // plotWorldState("Prediction");

        // Rendering
        ImGui::Render();
        int display_w, display_h;
        glfwGetFramebufferSize(window, &display_w, &display_h);
        glViewport(0, 0, display_w, display_h);
        ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
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
