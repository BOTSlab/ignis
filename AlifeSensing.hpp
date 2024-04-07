#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "Angles.hpp"
#include "WorldState.hpp"
#include "Config.hpp"
#include "CommonTypes.hpp"

using namespace CommonTypes;

namespace AlifeSensing {

struct AlifeSensorReading {
    bool hit;
    double alpha;
};

// A map from robot index to it's sensor reading.
using MapOfSensorReadings = std::map<size_t, AlifeSensorReading>;

/**
 * Checks if a line segment intersects with a circle.
 *
 * @param segmentStart The starting point of the line segment.
 * @param segmentEnd The ending point of the line segment.
 * @param circleCenter The center of the circle.
 * @param circleRadius The radius of the circle.
 * @return True if the line segment intersects with the circle, false otherwise.
 */
bool segmentIntersectsCircle(Vec2 segmentStart, Vec2 segmentEnd, Vec2 circleCenter, double circleRadius)
{
    // Get the vector from the start of the segment to the circle center.
    Vec2 segmentToCircle = circleCenter - segmentStart;

    // Get the vector representing the segment.
    Vec2 segment = segmentEnd - segmentStart;

    // Get the projection of the segmentToCircle vector onto the segment vector.
    double projection = segmentToCircle.x * segment.x + segmentToCircle.y * segment.y;

    // If the projection is negative, then the circle is behind the segment.
    if (projection < 0)
        return false;

    // Get the squared length of the segment.
    double segmentLengthSquared = segment.x * segment.x + segment.y * segment.y;

    // If the projection is greater than the squared length of the segment, then the circle is beyond the segment.
    if (projection > segmentLengthSquared)
        return false;

    // Get the squared distance from the circle center to the segment.
    double distanceSquared = segmentToCircle.x * segmentToCircle.x + segmentToCircle.y * segmentToCircle.y - projection * projection / segmentLengthSquared;

    // If the squared distance is less than the squared radius, then the segment intersects the circle.
    return distanceSquared < circleRadius * circleRadius;
}

/**
 * Gets sensor readings for all robots in the world.
 *
 * @param worldState A shared pointer to the current world state.
 * @return A map of robot indices to their corresponding sensor readings.
 */
MapOfSensorReadings allRobotsSense(std::shared_ptr<WorldState> worldState)
{
    MapOfSensorReadings robotIndexToSensorReadings;

    // Loop through all robots and get their sensor readings.
    for (size_t robotIndex = 0; robotIndex < worldState->robots.size(); robotIndex++)
    {   
        Robot &robot = worldState->robots[robotIndex];

        Vec2 segmentStart = robot.pos + Vec2(cos(robot.theta), sin(robot.theta)) * (robot.radius + config.segmentSensorOffset);
        Vec2 segmentEnd = segmentStart + Vec2(cos(robot.theta), sin(robot.theta)) * config.segmentSensorLength;

        bool hitPuck = false;
        for (const CircleBody &puck : worldState->pucks) {
            if (segmentIntersectsCircle(segmentStart, segmentEnd, puck.pos, puck.radius)) {
                hitPuck = true;
                break;
            }
        }

        // Determine alpha, the angle between the robot's heading and the local
        // goal direction.
        double angleToGoal = atan2(config.puckGoalY - robot.pos.y, config.puckGoalX - robot.pos.x);
        double alpha = Angles::getSmallestSignedAngularDifference(angleToGoal, robot.theta);

        robotIndexToSensorReadings[robotIndex] = {hitPuck, alpha};
    }

    return robotIndexToSensorReadings;
}

}; // namespace AlifeSensing