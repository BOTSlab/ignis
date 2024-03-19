#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "WorldState.hpp"
#include "WorldConfig.hpp"
#include "CurveGeneration.hpp"

namespace Following {
    void updateControlInputs(std::shared_ptr<WorldState> worldState, CurveGeneration::MapOfCurves robotIndexToBestCurves) {

        for (int i = 0; i < config.numberOfRobots; ++i) {
            cout << "i: " << i << "\n";
            if (robotIndexToBestCurves.find(i) == robotIndexToBestCurves.end()) {
                // For whatever reason, we don't have a curve for this robot.
                worldState->robots[i].controlInput = {0, 0};
                cout << "No curve for robot " << i << endl;
                continue;
            }

            // Choose the point at the nose of the robot.
            double noseX = worldState->robots[i].x + config.robotRadius * cos(worldState->robots[i].theta);
            double noseY = worldState->robots[i].y + config.robotRadius * sin(worldState->robots[i].theta);

            // Find the point on the curve closest to (noseX, noseY).
            double minDistance = std::numeric_limits<double>::max();
            int closestIndex = 0;
            auto &curve = robotIndexToBestCurves[i];
            for (int j = 0; j < curve.poses.size(); ++j) {
                double dx = curve.poses[j].x - noseX;
                double dy = curve.poses[j].y - noseY;
                double distance = std::sqrt(dx * dx + dy * dy);
                if (distance < minDistance) {
                    minDistance = distance;
                    closestIndex = j;
                }
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

            //double angularSpeed = config.maxAngularSpeed * angleDifference / M_PI;
            double angularSpeed = config.maxAngularSpeed * (angleDifference > 0 ? 1 : -1);
            worldState->robots[i].controlInput = {config.maxForwardSpeed, angularSpeed};
        }
    }
};