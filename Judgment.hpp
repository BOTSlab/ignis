#pragma once
#include <vector>
#include <cmath>
#include "Sim.hpp"
#include "WorldConfig.hpp"
#include "WorldState.hpp"
#include "Track.hpp"

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

//void judgeTrack(std::shared_ptr<Track> track, int robotIndex, std::shared_ptr<WorldState> worldState) {
void judgeTrack(Track& track, int robotIndex, std::shared_ptr<WorldState> worldState) {
    // Reset the number of collisions.  The actual world state that worldState
    // was copied from has its own history of collisions, but we want to judge
    // the track as if it were the first time the robot has moved along it.  
    worldState->resetCollisionCounts();

    // We'll remove all other robots from the scene so that any collisions they
    // have don't affect the score of the track.
    auto robot = worldState->robots[robotIndex];
    worldState->robots.clear();
    worldState->robots.push_back(robot);

    for (int i = 0; i < worldState->robots.size(); ++i) {
        if (i != robotIndex) {
            worldState->robots[i].vx = 0;
            worldState->robots[i].vy = 0;
        }
    }

    double beforeDistance = averagePuckDistanceToGoal(worldState, config.puckGoalX, config.puckGoalY);

    // Move the robot at robotIndex along the track.
    for (int i = 0; i < track.poses.size(); ++i) {
        worldState->robots[robotIndex].x = track.poses[i].x;
        worldState->robots[robotIndex].y = track.poses[i].y;
        //worldState->robots[robotIndex].theta = std::atan2(track->points[i].y - track->points[i - 1].y, track->points[i].x - track->points[i - 1].x);
        worldState->robots[robotIndex].theta = track.poses[i].theta;

        // Call sim update
        Sim::update(worldState);
    }

    double afterDistance = averagePuckDistanceToGoal(worldState, config.puckGoalX, config.puckGoalY);

    if (worldState->nRobotRobotCollisions == 0 && worldState->nRobotBoundaryCollisions == 0) {
        //track.score = track.baseScore + (beforeDistance - afterDistance) / track.poses.size();
        track.score = (beforeDistance - afterDistance);
        std::cout << "beforeDistance: " << beforeDistance << " afterDistance: " << afterDistance << " score: " << track.score << std::endl;        
    } else {
        std::cout << "worldState->nRobotRobotCollisions: " << worldState->nRobotRobotCollisions << std::endl;
        std::cout << "worldState->nRobotBoundaryCollisions: " << worldState->nRobotBoundaryCollisions << std::endl;
        track.score = -1;
    }
}

}; // namespace Judgment