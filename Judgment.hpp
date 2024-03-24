#pragma once
#include <vector>
#include <cmath>
#include "Following.hpp"
#include "Sim.hpp"
#include "WorldConfig.hpp"
#include "WorldState.hpp"
#include "CommonTypes.hpp"

using namespace CommonTypes;

using namespace Following;

namespace Judgment {

double averagePuckDistanceToGoal(std::shared_ptr<WorldState> worldState, double goalX, double goalY) {
    double totalDistance = 0.0;
    int puckCount = 0;

    for (const auto& puck : worldState->pucks) {
        double dx = puck.x - goalX;
        double dy = puck.y - goalY;
        totalDistance += std::sqrt(dx * dx + dy * dy);
        puckCount++;
    }

    double averageDistance = (puckCount > 0) ? totalDistance / puckCount : 0;
    return averageDistance;
}

double judgeCurve(const Curve& curve, int robotIndex, std::shared_ptr<WorldState> worldState) {
    // cout << "judgeCurve - robotIndex: " << robotIndex << endl;
    // Reset the number of collisions.  The actual world state that worldState
    // was copied from has its own history of collisions, but we want to judge
    // the curve as if it were the first time the robot has moved along it.  
    worldState->resetCollisionCounts();

    // We'll remove all other robots from the scene so that any collisions they
    // have don't affect the score of the curve.
    auto robot = worldState->robots[robotIndex];
    worldState->robots.clear();
    worldState->robots.push_back(robot);

    double beforeDistance = averagePuckDistanceToGoal(worldState, config.puckGoalX, config.puckGoalY);

    // Teleport the robot at robotIndex along the curve.
    /*
    for (int i = 0; i < curve.poses.size(); ++i) {
        worldState->robots[robotIndex].x = curve.poses[i].x;
        worldState->robots[robotIndex].y = curve.poses[i].y;
        //worldState->robots[robotIndex].theta = std::atan2(curve->points[i].y - curve->points[i - 1].y, curve->points[i].x - curve->points[i - 1].x);
        worldState->robots[robotIndex].theta = curve.poses[i].theta;

        // Call sim update
        Sim::update(worldState);
    }
    */

    // Simulate the robot moving along the curve.  Since following the curve
    // will consume it, first make a copy to study.
    Curve curveCopy = curve;
    int stepsSinceCurveShrank = 0;
    int lastPoseCount = curveCopy.poses.size();
    while (curveCopy.poses.size() > 0) {
        //cout << "curveCopy.poses.size(): " << curveCopy.poses.size() << endl;
        Sim::update(worldState);
        Following::updateControlInput(worldState, robotIndex, curveCopy);
        if (curveCopy.poses.size() != lastPoseCount) {
            lastPoseCount = curveCopy.poses.size();
            stepsSinceCurveShrank = 0;
        } else {
            stepsSinceCurveShrank++;
        }
        if (stepsSinceCurveShrank > config.maxStallSteps) {
            cout << "Curve has not shrunk in " << config.maxStallSteps << " steps.  Breaking." << endl;
            break;
        }
    }

    double afterDistance = averagePuckDistanceToGoal(worldState, config.puckGoalX, config.puckGoalY);

    if (stepsSinceCurveShrank > config.maxStallSteps) {
        return -10;
    } else if (worldState->nRobotRobotCollisions > 0 || worldState->nRobotBoundaryCollisions > 0) {
        //std::cout << "worldState->nRobotRobotCollisions: " << worldState->nRobotRobotCollisions << std::endl;
        //std::cout << "worldState->nRobotBoundaryCollisions: " << worldState->nRobotBoundaryCollisions << std::endl;
        return -1;
        
    } else {
        //std::cout << "beforeDistance: " << beforeDistance << " afterDistance: " << afterDistance << std::endl;
        return (beforeDistance - afterDistance);
    }
}

}; // namespace Judgment