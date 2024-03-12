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

#include "Ignis.hpp"
#include "CommonTypes.hpp"

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

void perRobotPlots(size_t robotIndex, const Ignis &ignis, double scaleFactor)
{
    auto worldState = ignis.simWorldState;
    auto config = ignis.config;
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

    /*
    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 1, ImPlot::GetColormapColor(4), IMPLOT_AUTO, ImPlot::GetColormapColor(4));
    ImPlot::PlotScatter("Tracks", allTrackXs, allTrackYs, nPointsInAllTracks);

    ImPlot::SetNextMarkerStyle(ImPlotMarker_Asterisk, 3, ImPlot::GetColormapColor(5), IMPLOT_AUTO, ImPlot::GetColormapColor(5));
    ImPlot::PlotScatter("Best Track", bestTrackXs, bestTrackYs, nPointsInBestTracks);

    ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, 1, ImPlot::GetColormapColor(6), IMPLOT_AUTO, ImPlot::GetColormapColor(6));
    ImPlot::PlotScatter("Plan", planXs, planYs, nPointsInAllPlans);
    */

    /*
    for (int i = 0; i < nr; ++i) {
        for (auto& track : robotIndexToTracks->at(i)) {
            std::ostringstream oss;
            oss << track.score;
            ImPlot::PlotText(oss.str().c_str(), track.poses.back().x, track.poses.back().y);
        }
    }
    */

    if (ignis.robotIndexToDilatedPolygonsMap.count(robotIndex) > 0) {
        const auto &dilatedPolygons = ignis.robotIndexToDilatedPolygonsMap.at(robotIndex);
        for (const Voronoi::DilatedPolygon &dilatedPolygon : dilatedPolygons)
        {
            std::vector<double> polygonXs;
            std::vector<double> polygonYs;
            for (const CommonTypes::Vertex &vertex : dilatedPolygon.vertices)
            {
                polygonXs.push_back(vertex.x);
                polygonYs.push_back(vertex.y);
            }
            // std::ostringstream oss;
            // oss << dilatedPolygon.dilation;
            // ImPlot::PlotLine(oss.str().c_str(), polygonXs.data(), polygonYs.data(), polygonXs.size());
            if (dilatedPolygon.dilation == 0)
                ImPlot::PlotLine("Dilated Polygons", polygonXs.data(), polygonYs.data(), polygonXs.size(), ImPlotLineFlags_Loop);
            else
                ImPlot::PlotLine("Dilated Polygons", polygonXs.data(), polygonYs.data(), polygonXs.size(), ImPlotLineFlags_Loop);
        }
    }

    /*
    if (ignis.robotIndexToSkeletonsMap.count(robotIndex) > 0) {
        auto &skeletons = ignis.robotIndexToSkeletonsMap.at(robotIndex);
        for (const Voronoi::Skeleton &skeleton : skeletons)
        {
            int markerSize = 4;
            for (const CommonTypes::Vertex &vertex : skeleton.vertices)
            {
                ImPlot::SetNextMarkerStyle(ImPlotMarker_Circle, markerSize, color, IMPLOT_AUTO, color);
                ImPlot::PlotScatter(robotString.c_str(), &vertex.x, &vertex.y, 1);
                markerSize += 2;
            }
        }
    }
    */

    if (ignis.robotIndexToCurvesMap.count(robotIndex) > 0) {

        const auto &curves = ignis.robotIndexToCurvesMap.at(robotIndex);
        for (const Track &curve : curves)
        {
            std::vector<double> curveXs;
            std::vector<double> curveYs;
            for (const Pose &pose : curve.poses)
            {
                curveXs.push_back(pose.x);
                curveYs.push_back(pose.y);
            }
            ImPlot::PlotLine(robotString.c_str(), curveXs.data(), curveYs.data(), curveXs.size());

            std::ostringstream oss;
            oss << curve.score;
            ImPlot::PlotText(oss.str().c_str(), curveXs.back(), curveYs.back());
        }
    }

    if (ignis.robotIndexToBestCurveMap.count(robotIndex) > 0) {

        std::vector<double> curveXs;
        std::vector<double> curveYs;
        const Track &bestCurve = ignis.robotIndexToBestCurveMap.at(robotIndex);
        for (const Pose &pose : bestCurve.poses)
        {
            curveXs.push_back(pose.x);
            curveYs.push_back(pose.y);
        }
        ImPlot::PlotLine("Best Curves", curveXs.data(), curveYs.data(), curveXs.size());
    }
}

void plotWorldState(const char *title, const Ignis &ignis)
{
    auto worldState = ignis.simWorldState;
    auto config = ignis.config;
    double noseRadius = 0.1 * config.robotRadius;

    // As above, but for robots.  Also get coordinates for the "noses" of the
    // robots, which are the points at the front of the robot, and are used to
    // draw the direction of the robot.
    /*
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
    */

    // Now get the x- and y-coordinates for the tracks.  First figure out how
    // many total points are there for all robots.  We are going to plot the
    // tracks for all robots together.
    /*
    auto robotIndexToTracks = ignis.robotIndexToTracks;
    int nPointsInAllTracks = 0;
    int nPointsInBestTracks = 0;
    for (int i = 0; i < nr; ++i) {
        for (auto& track : robotIndexToTracks->at(i)) {
            nPointsInAllTracks += track.poses.size();
            if (track.best)
                nPointsInBestTracks += track.poses.size();
        }
    }
    double* allTrackXs = new double[nPointsInAllTracks];
    double* allTrackYs = new double[nPointsInAllTracks];
    double* bestTrackXs = new double[nPointsInBestTracks];
    double* bestTrackYs = new double[nPointsInBestTracks];
    int allTracksIndex = 0, bestTrackIndex = 0;
    for (int i = 0; i < nr; ++i) {
        for (auto& track : robotIndexToTracks->at(i)) {
            for (auto& point : track.poses) {
                allTrackXs[allTracksIndex] = point.x;
                allTrackYs[allTracksIndex] = point.y;
                allTracksIndex++;
                if (track.best) {
                    bestTrackXs[bestTrackIndex] = point.x;
                    bestTrackYs[bestTrackIndex] = point.y;
                    bestTrackIndex++;
                }
            }
        }
    }
    //printf("nPointsInAllTracks: %d\n", nPointsInAllTracks);
    //printf("i: %d\n", i);

    // Get the x- and y-coordinates for each robot's plan.
    auto robotIndexToPlans = ignis.robotIndexToPlans;
    int nPointsInAllPlans = 0;
    for (int i = 0; i < nr; ++i) {
        nPointsInAllPlans += robotIndexToPlans->at(i).poses.size();
    }
    double* planXs = new double[nPointsInAllPlans];
    double* planYs = new double[nPointsInAllPlans];
    int planIndex = 0;
    for (int i = 0; i < nr; ++i) {
        for (auto& point : robotIndexToPlans->at(i).poses) {
            planXs[planIndex] = point.x;
            planYs[planIndex] = point.y;
            planIndex++;
        }
    }
    */

    double boundaryXs[] = {0, static_cast<double>(config.width), static_cast<double>(config.width), 0};
    double boundaryYs[] = {0, 0, static_cast<double>(config.height), static_cast<double>(config.height)};

    ImGui::Begin(title);
    double buffer = 100;
    ImPlot::SetNextAxesLimits(-buffer, config.width + buffer, -buffer, config.height + buffer);
    if (ImPlot::BeginPlot(title, ImVec2(config.width, config.height), ImPlotFlags_::ImPlotFlags_Equal | ImPlotFlags_::ImPlotFlags_NoTitle))
    {
        ImPlot::SetupLegend(ImPlotLocation_South, ImPlotLegendFlags_Outside);

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
            perRobotPlots(i, ignis, scaleFactor);

        plotInteraction(scaleFactor, config, worldState);

        ImPlot::EndPlot();
    }
    ImGui::End();

    //    delete [] puckXs;
    //    delete [] puckYs;
    /*
    delete [] robotXs;
    delete [] robotYs;
    delete [] robotNoseXs;
    delete [] robotNoseYs;
    */
    /*
    delete [] allTrackXs;
    delete [] allTrackYs;
    delete [] planXs;
    delete [] planYs;
    */
}

void handleControlsWindow(Ignis &ignis, ImGuiIO &io)
{
    static float forwardSpeed = 0.0;
    static float angularSpeed = 0.0;

    ImGui::Begin("Controls");

    if (ImGui::Button("Reset"))
        ignis.prepareToReset();

    if (!ignis.isPaused() && ImGui::Button("Pause"))
        ignis.prepareToPause();

    if (ignis.isPaused() && ImGui::Button("Play"))
        ignis.prepareToUnpause();

    if (ignis.isPaused() && ImGui::Button("Step"))
        ignis.prepareToStepOnce();

    // ImGui::SameLine();

    ImGui::Text("Application average %.3f ms/frame (%.1f FPS)", 1000.0f / io.Framerate, io.Framerate);
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
    GLFWwindow *window = glfwCreateWindow(1280, 720, "Ignis", nullptr, nullptr);
    if (window == nullptr)
        return 1;
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

    Ignis ignis;

    // Main loop
    while (!glfwWindowShouldClose(window))
    {
        ignis.step();

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

        handleControlsWindow(ignis, io);

        plotWorldState("Simulation", ignis);
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
