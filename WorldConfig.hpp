#pragma once
#include "CommonTypes.hpp"

enum class OverallMethod
{
    DilatedPolygonCurves,
    ArcCurves
};

enum class CurveBlendMethod
{
    Bulge
};

struct WorldConfig
{
    const OverallMethod overallMethod = OverallMethod::DilatedPolygonCurves;
    const CurveBlendMethod curveBlendMethod = CurveBlendMethod::Bulge;

    const int width = 800;
    const int height = 600;
    const int coldStartSteps = 100;
    const int numberOfRobots = 4;
    const int numberOfPucks = 0;
    const double robotRadius = 30;
    const double puckRadius = 25;
    const double maxForwardSpeed = 0.1;
    const double maxAngularSpeed = 1.5; //0.1;

    // For curve judgment.
    const double puckGoalX = 400;
    const double puckGoalY = 300;
    const int maxStallSteps = 1000;

    // For dilation of Voronoi cells.
    const double dilationDelta = 10;

    // For generating arc-curves.
    const double maxTrackLength = 400;
    const double sampleSpacing = 50;

    // For sensing.
    const double curveThickness = 20;
    const double forwardOffset = robotRadius;
    const double maxLateralOffset = 0.5 * robotRadius;
    const std::vector<CommonTypes::SensorPosition> sensorPositions = {
        {"Left", forwardOffset, -maxLateralOffset},
        {"CentreLeft", forwardOffset, -0.5*maxLateralOffset},
        {"CentreRight", forwardOffset, 0.5*maxLateralOffset},
        {"Right", forwardOffset, maxLateralOffset}
    };
} config;