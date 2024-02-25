#pragma once
#include <cmath>

struct WorldConfig
{
    const int width = 800;
    const int height = 600;
    const int numberOfRobots = 5;
    const int numberOfPucks = 0;
    const double robotRadius = 50;
    const double puckRadius = 25;

    const int numberOfTracks = 8;
    const double goalHalfAngleRange = M_PI/2;
    const double goalDistance = 100;
    const int maxTrackPoints = 100;;
} config;