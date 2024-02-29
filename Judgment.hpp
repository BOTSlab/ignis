#pragma once
#include "WorldConfig.hpp"
#include "Track.hpp"
#include <vector>

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

void judgeTrack(std::shared_ptr<Track> track, int robotIndex, std::shared_ptr<WorldState> worldState) {

    // If the track is re-generated any time, then there's no need for any
    // initialization here.
    // track->score = 0;

    // Reset the number of collisions.  The actual world state that worldState
    // was copied from has its own history of collisions, but we want to judge
    // the track as if it were the first time the robot has moved along it.  
    worldState->resetCollisionCounts();

    double beforeDistance = averagePuckDistanceToGoal(worldState, config.puckGoalX, config.puckGoalY);

    // Move the robot at robotIndex along the track.
    for (int i = 1; i < track->points.size(); ++i) {
        worldState->robots[robotIndex].x = track->points[i].x;
        worldState->robots[robotIndex].y = track->points[i].y;
        worldState->robots[robotIndex].theta = std::atan2(track->points[i].y - track->points[i - 1].y, track->points[i].x - track->points[i - 1].x);

        // Call sim update
        Sim::update(worldState);

        // track->score -= worldState->robots[robotIndex].nCollisions;
    }

    double afterDistance = averagePuckDistanceToGoal(worldState, config.puckGoalX, config.puckGoalY);

    if (worldState->nRobotRobotCollisions == 0 && worldState->nRobotBoundaryCollisions == 0) {
        track->score += beforeDistance - afterDistance;
    } else {
        track->score = -1;
    }

    // Normalize the score by the number of points on the track.
    // track->score /= track->points.size();
}

}; // namespace Judgment