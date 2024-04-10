#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "Angles.hpp"
#include "WorldState.hpp"
#include "Config.hpp"
#include "CommonTypes.hpp"
#include "Utils.hpp"

using namespace CommonTypes;

namespace AlifeSensingVersion2 {

struct AlifeSensorReadingVersion2 {
    bool hitPuck, hitRobot;
    double alpha;
};

// A map from robot index to it's sensor reading.
using MapOfSensorReadingsVersion2 = std::map<size_t, AlifeSensorReadingVersion2>;

/**
 * Gets sensor readings for all robots in the world.
 *
 * @param worldState A shared pointer to the current world state.
 * @return A map of robot indices to their corresponding sensor readings.
 */
MapOfSensorReadingsVersion2 allRobotsSense(std::shared_ptr<WorldState> worldState)
{
    MapOfSensorReadingsVersion2 robotIndexToSensorReadings;

    // Loop through all robots and get their sensor readings.
    for (size_t robotIndex = 0; robotIndex < worldState->robots.size(); robotIndex++)
    {   
        Robot &robot = worldState->robots[robotIndex];

        if (robot.slowedCounter > 0) {
            // A hack to speed up the simulation.  Slowed robots don't sense anything.
            robotIndexToSensorReadings[robotIndex] = {0, 0};
            continue;
        }

        Vec2 segmentStart = robot.pos + Vec2(cos(robot.theta), sin(robot.theta)) * (robot.radius + config.segmentSensorOffset);
        Vec2 segmentEnd = segmentStart + Vec2(cos(robot.theta), sin(robot.theta)) * config.segmentSensorLength;

        // Find the closest puck that the segment intersects.
        bool hitPuck = false;
        for (const CircleBody &puck : worldState->pucks) {
            if (Utils::segmentIntersectsCircle(segmentStart, segmentEnd, puck.pos, puck.radius)) {
                hitPuck = true;
                break;
            }
        }

        // Find the closest other robot that the segment intersects.
        bool hitRobot = false;
        for (size_t otherRobotIndex = 0; otherRobotIndex < worldState->robots.size(); otherRobotIndex++) {
            if (otherRobotIndex == robotIndex)
                continue;

            Robot &otherRobot = worldState->robots[otherRobotIndex];
            if (Utils::segmentIntersectsCircle(segmentStart, segmentEnd, otherRobot.pos, otherRobot.radius)) {
                hitRobot = true;
                break;
            }
        }

        // Determine alpha, the angle between the robot's heading and the local
        // goal direction.
        double angleToGoal = atan2(config.puckGoalY - robot.pos.y, config.puckGoalX - robot.pos.x);
        double alpha = Angles::getSmallestSignedAngularDifference(angleToGoal, robot.theta);

        robotIndexToSensorReadings[robotIndex] = {hitPuck, hitRobot, alpha};
    }

    return robotIndexToSensorReadings;
}

}; // namespace AlifeSensingVersion2