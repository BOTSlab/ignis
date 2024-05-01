#pragma once
#include <vector>
#include <cmath>
#include "ignis/CurveFollowing.hpp"
#include "common/Sim.hpp"
#include "common/Config.hpp"
#include "common/WorldState.hpp"
#include "ignis/CurveTypes.hpp"

using namespace CurveTypes;

using namespace CurveFollowing;

namespace Judgment {

double averagePuckDistanceToGoal(std::shared_ptr<WorldState> worldState, double goalX, double goalY) {
    double totalDistance = 0.0;
    int puckCount = 0;

    for (const auto& puck : worldState->pucks) {
        double dx = puck.pos.x - goalX;
        double dy = puck.pos.y - goalY;
        totalDistance += std::sqrt(dx * dx + dy * dy);
        puckCount++;
    }

    double averageDistance = (puckCount > 0) ? totalDistance / puckCount : 0;
    return averageDistance;
}

/*
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
    //
    //for (int i = 0; i < curve.poses.size(); ++i) {
    //    worldState->robots[robotIndex].x = curve.poses[i].x;
    //    worldState->robots[robotIndex].y = curve.poses[i].y;
    //    //worldState->robots[robotIndex].theta = std::atan2(curve->points[i].y - curve->points[i - 1].y, curve->points[i].x - curve->points[i - 1].x);
    //    worldState->robots[robotIndex].theta = curve.poses[i].theta;
    //
    //    // Call sim update
    //    Sim::update(worldState);
    //}

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
*/

// Judges the curve by assigning scores to each point.  
void judgeCurve(Curve& curve, int robotIndex, std::shared_ptr<WorldState> worldState) {
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

    int stepsSinceCurveShrank = 0;
    int lastIndexToSeek = 0;
    bool exit = false;
    double lastAPD = averagePuckDistanceToGoal(worldState, worldState->goalPos.x, worldState->goalPos.y);
    while (!exit && !curve.isFinishedFollowing()) {
// std::cout << "curve.getIndexToSeek(): " << curve.getIndexToSeek() << std::endl;

        Sim::update(worldState);
        CurveFollowing::updateControlInput(worldState, robotIndex, curve);

        double apd = averagePuckDistanceToGoal(worldState, worldState->goalPos.x, worldState->goalPos.y);

        double score = 0;
        if (stepsSinceCurveShrank > config.maxStallSteps) {
            std::cout << "Curve has not shrunk in " << config.maxStallSteps << " steps." << std::endl;
            exit = true;
            score = -10;
        } else if (worldState->nRobotRobotCollisions > 0 || worldState->nRobotBoundaryCollisions > 0) {
            // std::cout << "Robot collided with another robot, boundary, or a puck.  Breaking." << std::endl;
            score = -5;
        } else {
            score = lastAPD - apd;
        }

        // Assign score, understanding that this score may be overwrritten
        // if the simulated robot is still seeking the same point on the next iteration.
        if (!curve.isFinishedFollowing())
            curve.points[curve.getIndexToSeek()].score = score;

        if (curve.getIndexToSeek() != lastIndexToSeek) {
            lastIndexToSeek = curve.getIndexToSeek();
            stepsSinceCurveShrank = 0;
        } else {
            stepsSinceCurveShrank++;
        }

        lastAPD = apd;
    }

// Print out curve's contents
//for (const auto& point : curve.points)
//    std::cout << "Score: " << point.score << std::endl;

}

// Judges all curves and return the best curve for this robot.
Curve judgeCurves(int robotIndex, std::vector<Curve> &inputCurves, std::shared_ptr<WorldState> worldState)
{
    if (inputCurves.empty())
        throw std::runtime_error("No curves to judge.");

    // Judge all curves.
    for (auto& inputCurve : inputCurves) {
        // Judge the curve using a copy of the world state.
        auto worldStateToJudge = std::make_shared<WorldState>(*worldState);
        Judgment::judgeCurve(inputCurve, robotIndex, worldStateToJudge);
    }

    // Now find the best curve for this robot.
    double maxScore = -std::numeric_limits<double>::max();
    int bestCurveIndex = -1;
    for (int i = 0; i < inputCurves.size(); ++i) {
        double curveScore = inputCurves[i].getTotalScore();
        if (curveScore > maxScore) {
            maxScore = curveScore;
            bestCurveIndex = i;
        }
    }
    if (bestCurveIndex == -1)
        throw std::runtime_error("No best curve found.");
    std::cout << "judgeCurves - bestCurve score: " << maxScore << " length: " << inputCurves[bestCurveIndex].points.size() << std::endl;

    return inputCurves[bestCurveIndex];
}

}; // namespace Judgment