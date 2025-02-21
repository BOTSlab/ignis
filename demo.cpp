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

#include "forage/ForageScenario.hpp"
#include "common/CommonTypes.hpp"
#include "common/DataLogger.hpp"

const ImVec4 red = ImVec4(1.0f, 0.0f, 0.0f, 1.00f);
const ImVec4 green = ImVec4(0.0f, 1.0f, 0.0f, 1.00f);
const ImVec4 blue = ImVec4(0.0f, 0.0f, 1.0f, 1.00f);
const ImVec4 gray = ImVec4(0.7f, 0.7f, 0.7f, 1.00f);
const ImVec4 clear = ImVec4(0.0f, 0.0f, 0.0f, 0.0f);

// Global variables.
int selectedRobotIndex = -1;
int loopsPerUpdate = 1;

static void glfw_error_callback(int error, const char *description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

void plotInteraction(double scaleFactor, Config &config, std::shared_ptr<WorldState> &worldState)
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
            double robotX = worldState->robots[i].pos.x;
            double robotY = worldState->robots[i].pos.y;
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
        double robotX = worldState->robots[selectedRobotIndex].pos.x;
        double robotY = worldState->robots[selectedRobotIndex].pos.y;
        ImPlot::PlotScatter("Selected Robot", &robotX, &robotY, 1);

        // The selected robot will be manually controlled, with zero speed by default.
        worldState->robots[selectedRobotIndex].controlInput.forwardSpeed = 0;
        worldState->robots[selectedRobotIndex].controlInput.angularSpeed = 0;

        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_UpArrow)))
        {
            worldState->robots[selectedRobotIndex].controlInput.forwardSpeed = config.maxForwardSpeed;
        }
        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_DownArrow)))
        {
            worldState->robots[selectedRobotIndex].controlInput.forwardSpeed = -config.maxForwardSpeed;
        }
        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_LeftArrow)))
        {
            worldState->robots[selectedRobotIndex].controlInput.angularSpeed = config.maxAngularSpeed;
        }
        if (ImGui::IsKeyPressed(ImGui::GetKeyIndex(ImGuiKey_RightArrow)))
        {
            worldState->robots[selectedRobotIndex].controlInput.angularSpeed = -config.maxAngularSpeed;
        }
    }
}

void perRobotPlots(size_t robotIndex, const ForageScenario &forageScenario, double scaleFactor)
{
//cout << "perRobotPlots - START" << endl;
    Config &config = Config::getInstance();
    auto worldState = forageScenario.simWorldState;
    double noseRadius = 0.1 * config.robotRadius;
    auto color = ImPlot::GetColormapColor(robotIndex + 2);

    auto &robot = worldState->robots[robotIndex];

    ImPlot::PushStyleVar(ImPlotStyleVar_FillAlpha, 0.25f);
    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, scaleFactor * config.robotRadius, color, IMPLOT_AUTO, color);
    std::ostringstream oss;
    oss << "Robot " << robotIndex;
    std::string robotString = oss.str();
    ImPlot::PlotScatter(robotString.c_str(), &robot.pos.x, &robot.pos.y, 1);

    double noseX = robot.pos.x + (config.robotRadius - noseRadius) * cos(robot.theta);
    double noseY = robot.pos.y + (config.robotRadius - noseRadius) * sin(robot.theta);
    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, scaleFactor * noseRadius, color, IMPLOT_AUTO, color);
    ImPlot::PlotScatter(robotString.c_str(), &noseX, &noseY, 1);
    ImPlot::PopStyleVar();

    // Show the robot's forward heading using a dashed line.
    double infX = robot.pos.x + 2*robot.radius * cos(robot.theta);
    double infY = robot.pos.y + 2*robot.radius * sin(robot.theta);
    std::vector<double> xs = {noseX, infX};
    std::vector<double> ys = {noseY, infY};
    ImPlot::SetNextLineStyle(gray, 0.5);
    ImPlot::PlotLine("Heading", xs.data(), ys.data(), 2);

    if (forageScenario.robotIndexToSensorReadingMap.count(robotIndex) > 0) {
        std::ostringstream oss;
        oss << "Sensors for robot " << robotIndex;
        double sensorSize = 0.1 * config.robotRadius;

        const ForageSensorReading &sensorReading = forageScenario.robotIndexToSensorReadingMap.at(robotIndex);
        std::vector<double> xs = {sensorReading.segmentStart.x, sensorReading.segmentEnd.x};
        std::vector<double> ys = {sensorReading.segmentStart.y, sensorReading.segmentEnd.y};

        if (sensorReading.hitValue == 0) {
            ImPlot::SetNextLineStyle(gray, 2);
        } else if (sensorReading.hitValue == 1) {
            ImPlot::SetNextLineStyle(green, 4);
        } else if (sensorReading.hitValue == 2) {
            ImPlot::SetNextLineStyle(red, 4);
        } else {
            throw std::runtime_error("Unknown hit value");
        }

        ImPlot::HideNextItem();
        ImPlot::PlotLine("SegmentSensors", xs.data(), ys.data(), 2);        
    }
//cout << "perRobotPlots - END" << endl;
}

void plotWorldState(const char *title, const ForageScenario &forageScenario)
{
    Config &config = Config::getInstance();
    auto worldState = forageScenario.simWorldState;
    double noseRadius = 0.1 * config.robotRadius;

    double boundaryXs[] = {0, static_cast<double>(config.width), static_cast<double>(config.width), 0};
    double boundaryYs[] = {0, 0, static_cast<double>(config.height), static_cast<double>(config.height)};

    ImGui::Begin(title);
    double buffer = 100;
    double plotScale = 1.35;
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

        // Draw the goal position.
        if (config.controlMethod != ControlMethod::ThreeParameterGauci) {
            ImPlot::SetNextMarkerStyle(ImPlotMarker_Cross, scaleFactor * 40, green, IMPLOT_AUTO, green);
            ImPlot::PlotScatter("Goal", &worldState->goalPos.x, &worldState->goalPos.y, 1);
        }

        std::vector<double> puckXs, puckYs;
        for (int i = 0; i < worldState->pucks.size(); ++i)
        {
            puckXs.push_back(worldState->pucks[i].pos.x);
            puckYs.push_back(worldState->pucks[i].pos.y);
        }
        ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, scaleFactor * config.puckRadius, ImPlot::GetColormapColor(1), IMPLOT_AUTO, ImPlot::GetColormapColor(1));
        ImPlot::PlotScatter("Pucks", puckXs.data(), puckYs.data(), puckXs.size());

        for (int i = 0; i < worldState->robots.size(); ++i)
            perRobotPlots(i, forageScenario, scaleFactor);

        plotInteraction(scaleFactor, config, worldState);

        ImPlot::EndPlot();
    }
    ImGui::End();
}

void handleControlsWindow(ForageScenario &forageScenario, ImGuiIO &io)
{
    static float forwardSpeed = 0.0;
    static float angularSpeed = 0.0;

    ImGui::Begin("Controls");

    if (ImGui::Button("Reset"))
        forageScenario.prepareToReset();

    if (!forageScenario.isPaused() && ImGui::Button("Pause"))
        forageScenario.prepareToPause();

    if (forageScenario.isPaused() && ImGui::Button("Play"))
        forageScenario.prepareToUnpause();

    if (forageScenario.isPaused() && ImGui::Button("Step"))
        forageScenario.prepareToStepOnce();

    ImGui::SliderInt("Loops per update", &loopsPerUpdate, 1, 100);

    // ImGui::SameLine();

    ImGui::Text("step count: %d", forageScenario.getStepCount());
    ImGui::Text("evaluation: %g", forageScenario.currentEvaluation);
    ImGui::Text("cum. eval.: %g", forageScenario.cumulativeEvaluation);
    ImGui::Text("steps with r-r collsions: %d", forageScenario.stepsWithRobotRobotCollisions);
    //ImGui::Text("r-r collsions: %d", forageScenario.simWorldState->nRobotRobotCollisions);
    //ImGui::Text("r-b collsions: %d", forageScenario.simWorldState->nRobotBoundaryCollisions);
    ImGui::Text("%.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
    ImGui::End();
}

// Main code
int main(int, char **)
{
    Config &config = Config::getInstance();

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
    GLFWwindow *window = glfwCreateWindow(2000, 850, "ForageScenario", nullptr, nullptr);
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

    // Read in the parameters in parameters.dat into parameters.vec
    /*
    string dir = "./";
    std::ifstream lastParametersFile(dir + "parameters.dat");
    if (lastParametersFile.is_open()) {
        parameters.vec.clear();
        double value;
        while (lastParametersFile >> value)
            parameters.vec.push_back(value);
        cout << "Read in the following parameters: ";
        for (int i = 0; i < parameters.vec.size(); ++i)
            cout << parameters.vec[i] << " ";
    } else {
        cout << "Could not open last_parameters.dat.  Using default parameters." << endl;
    }
    */

    DataLogger dataLogger(0, "data/demo/");
    ForageScenario forageScenario(0);

    // Main loop
    for (int loopCount = 0; !glfwWindowShouldClose(window); ++loopCount) {
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

        if (loopCount % loopsPerUpdate == 0) {
            //cout << "loopCount: " << loopCount << endl;
// HACK FOR PLOTTING: Force the goal position
//if (!forageScenario.isPaused()) {
//forageScenario.simWorldState->goalPos = Vec2(300, 150);
//cout << "goalPos: " << forageScenario.simWorldState->goalPos.x << ", " << forageScenario.simWorldState->goalPos.y << endl;
//}

            if (forageScenario.getStepCount() == config.stepsPerDemoRun)
                forageScenario.prepareToPause();

            forageScenario.step();
            
            int stepCount = forageScenario.getStepCount();
            if (!forageScenario.isPaused() && (stepCount == 0 || stepCount % 10 == 0))
                dataLogger.writeToFile(forageScenario.simWorldState, forageScenario.getStepCount(), forageScenario.currentEvaluation, forageScenario.cumulativeEvaluation);
        }

        handleControlsWindow(forageScenario, io);

        plotWorldState("Simulation", forageScenario);

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
