#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "WorldState.hpp"
#include "WorldConfig.hpp"
#include "Track.hpp"

namespace CurveFollowing {
    // Given the current state of the world and a map of robot indices to their
    // path tracks, set the robots' control inputs to follow their path tracks.
    void updateControlInputs(std::shared_ptr<WorldState> worldState, std::shared_ptr<VectorOfPlans> robotIndexToPlans) {

        for (int i = 0; i < config.numberOfRobots; ++i) {
            // Choose the point at the nose of the robot.
            double noseX = worldState->robots[i].x + config.robotRadius * cos(worldState->robots[i].theta);
            double noseY = worldState->robots[i].y + config.robotRadius * sin(worldState->robots[i].theta);

            // Find the point in the plan closest to (noseX, noseY).
            double minDistance = std::numeric_limits<double>::max();
            int closestIndex = 0;
            auto &plan = (*robotIndexToPlans)[i];
            for (int j = 0; j < plan.poses.size(); ++j) {
                double dx = plan.poses[j].x - noseX;
                double dy = plan.poses[j].y - noseY;
                double distance = std::sqrt(dx * dx + dy * dy);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestIndex = j;
                }
            }

            // Determine whether the the point closest to (noseX, noseY) lies to
            // the left or right of the line from the robot's current position
            // through its nose.
            double dx = plan.poses[closestIndex].x - worldState->robots[i].x;
            double dy = plan.poses[closestIndex].y - worldState->robots[i].y;
            double angle = std::atan2(dy, dx);
            double angleDifference = angle - worldState->robots[i].theta;
            while (angleDifference > M_PI) angleDifference -= 2 * M_PI;
            while (angleDifference < -M_PI) angleDifference += 2 * M_PI;

            //double angularSpeed = config.maxAngularSpeed * angleDifference / M_PI;
            double angularSpeed = config.maxAngularSpeed * (angleDifference > 0 ? 1 : -1);
            worldState->robots[i].controlInput = {config.maxForwardSpeed, angularSpeed};
        }
    }
};