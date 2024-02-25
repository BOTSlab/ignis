#pragma once
#include "WorldConfig.hpp"
#include "Track.hpp"
#include <vector>

namespace Judgment {

void judge(std::shared_ptr<Track> track, int robotIndex, std::shared_ptr<WorldState> worldState) {
    track->score = 0;

    // Move the robot at robotIndex along the track.
    for (int i = 1; i < track->points.size(); ++i) {

        // Make a copy of the actual world state
        // WorldState simWorldState = worldState;

        worldState->robots[robotIndex].x = track->points[i].x;
        worldState->robots[robotIndex].y = track->points[i].y;
        worldState->robots[robotIndex].theta = std::atan2(track->points[i].y - track->points[i - 1].y, track->points[i].x - track->points[i - 1].x);

        // Call sim update
        Sim::update(worldState);

        track->score -= worldState->robots[robotIndex].nCollisions;
    }
}

}; // namespace Judgment