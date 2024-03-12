#pragma once
#include <cmath>

struct WorldConfig
{
    const int width = 800;
    const int height = 600;
    const int numberOfRobots = 1;
    const int numberOfPucks = 0;
    const double robotRadius = 25;
    const double puckRadius = 50;
    const double maxForwardSpeed = 0.1;
    const double maxAngularSpeed = 0.1;

    // For track judgment.
    const double puckGoalX = 400;
    const double puckGoalY = 300;

    // For track generation.
    const int numberOfTracks = 9;
    const double maxTrackLength = 500;
    const double sampleSpacing = 10;

    // For plans.
    const int planMaxLength = 5;

    // For dilation of Voronoi cells.
    const double dilationDelta = 50;
} config;