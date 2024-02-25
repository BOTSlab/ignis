#pragma once
#include "Track.hpp"
#include "TrackGeneration.hpp"
#include "WorldConfig.hpp"
#include "WorldState.hpp"
#include "Judgment.hpp"
#include <cmath>
#include <map>
#include <utility>
#include <vector>

namespace Planning {

enum class PlanOperation {
    GetAllTracks,
    GetBestTrack
};

std::shared_ptr<Plan> plan(std::shared_ptr<WorldState> worldState, PlanOperation operation = PlanOperation::GetAllTracks)
{
    std::cout << "\nNEW PLAN\n"<< std::endl;
    auto plan = std::make_shared<Plan>();

    for (int i = 0; i < config.numberOfRobots; ++i) {
        double x = worldState->robots[i].x;
        double y = worldState->robots[i].y;
        double theta = worldState->robots[i].theta;

        // If we are looking for the best track, we will need to keep track of the
        // best track and its score as we go.
        std::shared_ptr<Track> bestTrack = nullptr;
        double bestScore = -std::numeric_limits<double>::max();

        // Choose a set of goal positions for the robot to reach.  They
        // will be distributed uniformly on an arc in front of the robot.
        double minAngle = -config.goalHalfAngleRange;
        double maxAngle = config.goalHalfAngleRange;
        for (int j = 0; j < config.numberOfTracks; ++j) {
            double angle = minAngle + (maxAngle - minAngle) * j / (config.numberOfTracks - 1);
            double goalX = x + config.goalDistance * cos(theta + angle);
            double goalY = y + config.goalDistance * sin(theta + angle);

            //std::shared_ptr<Track> track = TrackGeneration::smoothController1(x, y, theta, goalX, goalY);
            std::shared_ptr<Track> track = TrackGeneration::lookupTableTrack(x, y, theta, goalX, goalY);

            // Judge the track using a copy of the world state.
            auto worldStateToJudge = std::make_shared<WorldState>(*worldState);
            Judgment::judge(track, i, worldStateToJudge);

            // Add the track to the plan.
            if (operation == PlanOperation::GetBestTrack) {
                if (track->score > bestScore) {
                    std::cout << "new winning score " << track->score << std::endl;
                    bestScore = track->score;
                    bestTrack = track;
                }
            } else {
                (*plan)[i].push_back(*track); // BAD: This is a copy of the track, not a shared pointer.
            }
        }

        if (operation == PlanOperation::GetBestTrack)
            (*plan)[i].push_back(*bestTrack);

        std::cout << "Number of tracks in the plan for robot " << i << ": " << (*plan)[i].size() << std::endl;
    }

    return plan;
}

}; // namespace Planning