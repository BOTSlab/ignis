#pragma once
#include "CommonTypes.hpp"

enum class AlifeControlMethod
{
    EvolvedGauci,
    EvolvedCubic,
    EvolvedLinear,
    EvolvedSpinner
};

enum class OverallMethod
{
    DilatedPolygonCurves,
    ArcCurves
};

enum class CurveBlendMethod
{
    Bulge
};

struct Config
{
    const AlifeControlMethod controlMethod = AlifeControlMethod::EvolvedSpinner;

    const OverallMethod overallMethod = OverallMethod::DilatedPolygonCurves;
    const CurveBlendMethod curveBlendMethod = CurveBlendMethod::Bulge;

    const int width = 1200;
    const int height = 600;
    const int coldStartSteps = 0;
    const int numberOfRobots = 10;
    const int numberOfPucks = 25;
    const double robotRadius = 10;
    const double puckRadius = 20;
    const double maxForwardSpeed = 0.25;
    const double maxAngularSpeed = 0.1;

    // For curve judgment.
    //const double puckGoalX = 0;
    //const double puckGoalY = 600;
    const int maxStallSteps = 1000;

    // For dilation of Voronoi cells.
    const double dilationDelta = 10;

    // For generating arc-curves.
    const double maxTrackLength = 400;
    const double sampleSpacing = 50;

    // For line sensing.
    /*
    const double curveThickness = 20;
    const double forwardOffset = robotRadius;
    const double maxLateralOffset = 0.5 * robotRadius;
    const std::vector<CommonTypes::SensorPosition> sensorPositions = {
        {"Left", forwardOffset, -maxLateralOffset},
        {"CentreLeft", forwardOffset, -0.5*maxLateralOffset},
        {"CentreRight", forwardOffset, 0.5*maxLateralOffset},
        {"Right", forwardOffset, maxLateralOffset}
    };
    */

    // For Alife scenario.
    const double segmentSensorOffset = 0.1;
    const double segmentSensorLength = 1000000;
    const double slowedSteps = 0;
    const double minAngular = 0;
} config;