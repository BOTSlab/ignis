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

// Update the control input of robot i based on the given curve.  The curve will
// be "consumed" as the robot moves along it.  Any consumed poses will be
// removed from the curve and returned in the output.
std::vector<Pose> updateControlInput(std::shared_ptr<WorldState> worldState, int i, Curve &curve)
{
    std::vector<Pose> consumedPoses;

    // std::cout << "START updateControlInput for robot: " << i << "\n";
    if (curve.poses.size() == 0) {
        std::cerr << "updateControlInput - No poses in curve for robot " << i << std::endl;
        return consumedPoses;
    }

    // Choose the point at the nose of the robot.
    double noseX = worldState->robots[i].x + config.robotRadius * cos(worldState->robots[i].theta);
    double noseY = worldState->robots[i].y + config.robotRadius * sin(worldState->robots[i].theta);

    // Find the point on the curve closest to (noseX, noseY).
    double minDistance = std::numeric_limits<double>::max();
    int closestIndex = -1;
    for (int j = 0; j < curve.poses.size(); ++j) {
        double dx = curve.poses[j].x - noseX;
        double dy = curve.poses[j].y - noseY;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = j;
        }
    }
    if (closestIndex == -1) {
        std::cerr << "No closest point found on curve for robot " << i << std::endl;
        return consumedPoses;
    }

    // Determine whether the the point closest to (noseX, noseY) lies to
    // the left or right of the line from the robot's current position
    // through its nose.
    double dx = curve.poses[closestIndex].x - worldState->robots[i].x;
    double dy = curve.poses[closestIndex].y - worldState->robots[i].y;
    double angle = std::atan2(dy, dx);
    double angleDifference = angle - worldState->robots[i].theta;
    while (angleDifference > M_PI) angleDifference -= 2 * M_PI;
    while (angleDifference < -M_PI) angleDifference += 2 * M_PI;

    double angularSpeed = config.maxAngularSpeed * angleDifference / M_PI;
    //double angularSpeed = config.maxAngularSpeed * (angleDifference > 0 ? 1 : -1);
    worldState->robots[i].controlInput = {config.maxForwardSpeed, angularSpeed};

    // Consume the curve by the closest point and all preceding points.
    if (minDistance < config.robotRadius / 4) {
        // Add the poses to be removed to consumedPoses
        consumedPoses.insert(consumedPoses.end(), curve.poses.begin(), curve.poses.begin() + closestIndex + 1);
        curve.poses.erase(curve.poses.begin(), curve.poses.begin() + closestIndex + 1);
    }
    // std::cout << "END updateControlInput for robot: " << i << "\n";

    return consumedPoses;
}

void updateControlInputs(std::shared_ptr<WorldState> worldState, MapOfCurves &robotIndexToBestCurves)
{
    for (int i = 0; i < config.numberOfRobots; ++i)
    {
        if (robotIndexToBestCurves.find(i) == robotIndexToBestCurves.end()) {
            // For whatever reason, we don't have a curve for this robot.
            worldState->robots[i].controlInput = {0, 0};
            cout << "No curve for robot " << i << endl;
            continue;
        }

        updateControlInput(worldState, i, robotIndexToBestCurves.at(i));
    }
}

}; // namespace Following