#pragma once
#include <cmath>

struct WorldConfig
{
    const int width = 800;
    const int height = 600;
    const int numberOfRobots = 1;
    const int numberOfPucks = 50;
    const double robotRadius = 10;
    const double puckRadius = 25;
    const double maxForwardSpeed = 0.1;
    const double maxAngularSpeed = 0.1;

    // For curve judgment.
    const double puckGoalX = 400;
    const double puckGoalY = 300;
    const int maxStallSteps = 100;

    // For dilation of Voronoi cells.
    const double dilationDelta = 10;
} config;