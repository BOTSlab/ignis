#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "WorldState.hpp"
#include "WorldConfig.hpp"
#include "CommonTypes.hpp"

using namespace CommonTypes;

namespace Following {

// Update the control input of robot i based on the given curve.  The curve's
// indexToSeek value will be moved along the curve until it reaches the end.
void updateControlInput(std::shared_ptr<WorldState> worldState, int i, Curve &curve)
{
    if (curve.points.size() == 0) {
        std::cerr << "updateControlInput - No points in curve for robot " << i << std::endl;
        return;
    }
    if (curve.isFinishedFollowing()) {
        std::cerr << "updateControlInput - indexToSeek indicates curve already followed for robot " << i << std::endl;
        return;
    }

    // Choose the point at the nose of the robot.
    double noseX = worldState->robots[i].x + config.robotRadius * cos(worldState->robots[i].theta);
    double noseY = worldState->robots[i].y + config.robotRadius * sin(worldState->robots[i].theta);

    // Start at indexToSeek, find the point on the curve closest to (noseX, noseY).
    double minDistance = std::numeric_limits<double>::max();
    int closestIndex = -1;
    for (int j = curve.getIndexToSeek(); j < curve.points.size(); ++j) {
        double dx = curve.points[j].pose.x - noseX;
        double dy = curve.points[j].pose.y - noseY;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = j;
        }
    }
    if (closestIndex == -1) {
        std::cerr << "No closest point found on curve for robot " << i << std::endl;
        return;
    }

    // By setting indexToSeek to closestIndex, we're saying that this is the
    // point we're trying to reach.
    curve.setIndexToSeek(closestIndex);

    // Determine whether the point found above lies to the left or right of the 
    // line from the robot's current position through its nose.
    double dx = curve.points[closestIndex].pose.x - worldState->robots[i].x;
    double dy = curve.points[closestIndex].pose.y - worldState->robots[i].y;
    double angle = std::atan2(dy, dx);
    //double angleDifference = angle - worldState->robots[i].theta;
    double angleDifference = Angles::getSmallestSignedAngularDifference(angle, worldState->robots[i].theta);
    //while (angleDifference > M_PI) angleDifference -= 2 * M_PI;
    //while (angleDifference < -M_PI) angleDifference += 2 * M_PI;

    double angularSpeed = config.maxAngularSpeed * angleDifference / M_PI;
    //double angularSpeed = config.maxAngularSpeed * (angleDifference > 0 ? 1 : -1);
    worldState->robots[i].controlInput = {config.maxForwardSpeed, angularSpeed};
    
    // If we're close enough to the point, indicate that we've reached it by
    // advancing indexToSeek.  If we're seeking the last point, we're done.
    if (minDistance < config.robotRadius / 4 || closestIndex == curve.points.size() - 1)
        curve.advanceIndexToSeek();

//    if (Angles::getSmallestSignedAngularDifference(angle, worldState->robots[i].theta) < 0)
//        curve.advanceIndexToSeek();
}

void updateControlInputs(std::shared_ptr<WorldState> worldState, MapOfCurves &robotIndexToBestCurves)
{
    for (int i = 0; i < config.numberOfRobots; ++i)
    {
        if (robotIndexToBestCurves.find(i) == robotIndexToBestCurves.end()) {
            // For whatever reason, we don't have a curve for this robot.
            worldState->robots[i].controlInput = {0, 0};
            std::cout << "No curve for robot " << i << std::endl;
            continue;
        }

        updateControlInput(worldState, i, robotIndexToBestCurves.at(i));
    }
}

}; // namespace Following