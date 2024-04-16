#pragma once
#include "CommonTypes.hpp"

enum class AlifeControlMethod
{
    EvolvedGauci,
    EvolvedActiveVision,
    EvolvedActiveVisionPlusRandom
};

// enum class OverallMethod
// {
//     DilatedPolygonCurves,
//     ArcCurves
// };

// enum class CurveBlendMethod
// {
//     Bulge
// };

struct Config
{
    const AlifeControlMethod controlMethod = AlifeControlMethod::EvolvedGauci;

    //const OverallMethod overallMethod = OverallMethod::DilatedPolygonCurves;
    //const CurveBlendMethod curveBlendMethod = CurveBlendMethod::Bulge;

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
    //const int maxStallSteps = 1000;

    // For dilation of Voronoi cells.
    //const double dilationDelta = 10;

    // For generating arc-curves.
    //const double maxTrackLength = 400;
    //const double sampleSpacing = 50;

    // For optimize
    const unsigned int nGenerations = 1000;
    const unsigned int populationSize = 10;
    const unsigned int runsPerEvaluation = 10;
    const unsigned int stepsPerOptRun = 2000;

    // For demonstrate
    const unsigned int stepsPerDemoRun = 200;

    // For Alife scenario.
    const double segmentSensorOffset = 0.1;
    const double segmentSensorLength = 1000000;
    const double slowedSteps = 0;

} config;