#pragma once
#include "common/CommonTypes.hpp"

enum class ForageControlMethod
{
    EvolvedGauci,
    EvolvedActiveVision,
    EvolvedActiveVisionPlusRandom
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

enum class DilationOption
{
    JustZero,
    PuckRadius,
    DilationDelta
};


struct Config
{
    const ForageControlMethod controlMethod = ForageControlMethod::EvolvedActiveVision;

    const OverallMethod overallMethod = OverallMethod::DilatedPolygonCurves;
    const CurveBlendMethod curveBlendMethod = CurveBlendMethod::Bulge;
    const DilationOption dilationOption = DilationOption::DilationDelta;

    const int width = 1200;
    const int height = 600;
    const int coldStartSteps = 0;
    const int numberOfRobots = 8;
    const int numberOfPucks = 40;
    const double robotRadius = 10;
    const double puckRadius = 25;
    const double maxForwardSpeed = 0.25;
    const double maxAngularSpeed = 0.1;

    // For curve judgment.
    const int maxStallSteps = 1000;

    // For dilation of Voronoi cells.
    const double dilationDelta = 10;

    // For generating arc-curves.
    const double maxTrackLength = 400;
    const double sampleSpacing = 50;

    // For optimize
    const unsigned int nGenerations = 100;
    const unsigned int populationSize = 10;
    const unsigned int runsPerEvaluation = 10;
    const unsigned int stepsPerOptRun = 2000;

    // For demonstrate
    const unsigned int stepsPerDemoRun = 4000;

    // For Forage scenario.
    const double segmentSensorOffset = 0.1;
    const double segmentSensorLength = 10000;
    const double slowedSteps = 0;

    // For Vorlife scenario.
    const bool useVoronoi = false;
} config;