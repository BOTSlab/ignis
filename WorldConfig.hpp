#pragma once
#include <cmath>

struct WorldConfig
{
    const int width = 800;
    const int height = 600;
    const int numberOfRobots = 1;
    const int numberOfPucks = 2;
    const double robotRadius = 50;
    const double puckRadius = 25;
    const double maxForwardSpeed = 0.1;
    const double maxAngularSpeed = 0.05; // 0.01;

    const double puckGoalX = 400;
    const double puckGoalY = 300;

    const int numberOfTracks = 9;
    const double goalHalfAngleRange = M_PI/2;
    const double goalDistance = 100;
    const int maxTrackPoints = 100;;
} config;