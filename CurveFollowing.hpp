#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "WorldState.hpp"
#include "WorldConfig.hpp"
#include "Planning.hpp"

namespace CurveFollowing {
    // Given the current state of the world and a plan (with one track per robot),
    // set the robots' control inputs to follow the curve represented by the plan.
    void updateControlInputs(std::shared_ptr<WorldState> worldState, std::shared_ptr<Plan> plan) {

        for (int i = 0; i < config.numberOfRobots; ++i) {
            // Choose the point at the nose of the robot.
            double noseX = worldState->robots[i].x + config.robotRadius * cos(worldState->robots[i].theta);
            double noseY = worldState->robots[i].y + config.robotRadius * sin(worldState->robots[i].theta);

            // Find the point on the track closest to (noseX, noseY).
            double minDistance = std::numeric_limits<double>::max();
            int closestIndex = 0;
            Track &track = (*plan)[i][0];
            for (int j = 0; j < track.points.size(); ++j) {
                double dx = track.points[j].x - noseX;
                double dy = track.points[j].y - noseY;
                double distance = std::sqrt(dx * dx + dy * dy);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestIndex = j;
                }
            }

            // Determine whether the robot should turn left or right to get to the
            // closest point on the track.
            double dx = track.points[closestIndex].x - noseX;
            double dy = track.points[closestIndex].y - noseY;
            double angleToClosest = std::atan2(dy, dx);
            double angleDifference = angleToClosest - worldState->robots[i].theta;
            if (angleDifference > M_PI) angleDifference -= 2 * M_PI;
            if (angleDifference < -M_PI) angleDifference += 2 * M_PI;

            // Set the control input based on the angle difference.
            double angularSpeed = config.maxAngularSpeed * angleDifference;

            worldState->robots[i].controlInput = {config.maxForwardSpeed, angularSpeed};
        }
    }
};