#pragma once
#include "CommonTypes.hpp"
#include "WorldState.hpp"
#include "WorldConfig.hpp"

using namespace CommonTypes;

namespace ArcCurveGeneration {

Curve arcCurve(double startX, double startY, double startTheta, double radius) {

    Curve curve;

    if (radius == 0) {
        // If the radius is 0, then we are generating a straight line.
        for (double t = config.sampleSpacing; t < config.maxTrackLength; t += config.sampleSpacing) {
            double x = startX + t * cos(startTheta);
            double y = startY + t * sin(startTheta);
            curve.poses.push_back({x, y, startTheta});
        }
        return curve;
    }

    // If the radius is negative, then we know we are generating an arc on the right.
    bool left = radius > 0;

    // We'll now ensure the radius is positive.
    radius = fabs(radius);

    // Sample along the arc until we reach the maximum arc length.
    double x, y, theta;
    for (double arcLength = config.sampleSpacing; arcLength < config.maxTrackLength; arcLength += config.sampleSpacing) {

        double alpha = arcLength / radius;
        if (alpha > M_PI)
            break;

        if (left) {
            x = startX + radius * (cos(startTheta + M_PI/2.0) + cos(startTheta - M_PI/2.0 + alpha));
            y = startY + radius * (sin(startTheta + M_PI/2.0) + sin(startTheta - M_PI/2.0 + alpha));
            theta = startTheta + alpha;
        } else {
            x = startX + radius * (cos(startTheta - M_PI/2.0) + cos(startTheta + M_PI/2.0 - alpha));
            y = startY + radius * (sin(startTheta - M_PI/2.0) + sin(startTheta + M_PI/2.0 - alpha));
            theta = startTheta - alpha;
        }
        curve.poses.push_back({x, y, theta});
    }

    return curve;
}

std::vector<Curve> arcCurvesForRobot(Robot robot)
{
    int numberOfArcs = 50;
    double maxTurningRadius = 250;

    std::vector<Curve> curves;

    // Positive and negative turning radii.
    /*
    for (int j = 0; j < numberOfArcs; ++j)
    {
        double turningRadius = -maxTurningRadius + 2 * maxTurningRadius * j / (numberOfArcs - 1);
        curves.push_back( ArcCurveGeneration::arcCurve(robot.x, robot.y, robot.theta, turningRadius) );
    }
    */

    // Only positive turning radii, not including the straight-line case (0).
    for (int j = 1; j < numberOfArcs; ++j)
    {
        double turningRadius = maxTurningRadius * j / (numberOfArcs - 1);
        curves.push_back( ArcCurveGeneration::arcCurve(robot.x, robot.y, robot.theta, turningRadius) );
    }

    return curves;
}

}; // namespace ArcCurveGeneration