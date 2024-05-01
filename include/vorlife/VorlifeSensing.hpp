#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "common/Angles.hpp"
#include "common/WorldState.hpp"
#include "common/Config.hpp"
#include "common/CommonTypes.hpp"
#include "common/Utils.hpp"
#include "alife/Parameters.hpp"


using namespace CommonTypes;

namespace VorlifeSensing {

struct VorlifeSensorReading {
    Vec2 segmentStart, segmentEnd;
    int hitValue; // 0: no hit, 1: hit puck, 2: hit robot
};

// A map from robot index to it's sensor reading.
using MapOfSensorReadings = std::map<size_t, VorlifeSensorReading>;

std::vector<CircleBody*> getAllPucks(std::shared_ptr<WorldState> worldState)
{
    std::vector<CircleBody*> allPucks;
    for (CircleBody &puck : worldState->pucks)
        allPucks.push_back(&puck);
    return allPucks;
}

std::vector<CircleBody*> getVoronoiVisiblePucks(std::shared_ptr<WorldState> worldState, std::vector<DilatedPolygon> dilatedPolygons)
{
    // Assume there is only one polygon in dilatedPolygon
    DilatedPolygon dilatedPolygon = dilatedPolygons[0];

    // Go through all pucks and see if they lie within dilatedPolygon.  If so,
    // add them to the list of visible pucks.
    std::vector<CircleBody*> visiblePucks;
    for (CircleBody &puck : worldState->pucks) {
        if (dilatedPolygon.contains(puck.pos))
            visiblePucks.push_back(&puck);
    }
    return visiblePucks;
}

/**
 * Gets sensor readings for all robots in the world.
 *
 * @param worldState A shared pointer to the current world state.
 * @return A map of robot indices to their corresponding sensor readings.
 */
MapOfSensorReadings allRobotsSense(std::shared_ptr<WorldState> worldState,  GeosVoronoi::MapOfVectorOfDilatedPolygons &robotIndexToDilatedPolygonsMap)
{
    MapOfSensorReadings robotIndexToSensorReadings;

    // Loop through all robots and get their sensor readings.
    for (size_t robotIndex = 0; robotIndex < worldState->robots.size(); robotIndex++)
    {   
        Robot &robot = worldState->robots[robotIndex];

        if (robot.slowedCounter > 0) {
            // A hack to speed up the simulation.  Slowed robots don't sense anything.
            robotIndexToSensorReadings[robotIndex] = {0, 0};
            continue;
        }

        // Determine alpha, the angle between the robot's heading and the local
        // goal direction.
        double angleToGoal = atan2(worldState->goalPos.y - robot.pos.y, worldState->goalPos.x - robot.pos.x);
        double alpha = Angles::getSmallestSignedAngularDifference(angleToGoal, robot.theta);

        // This is the active vision component, which can modify the sensor's angle.
        double sensorAngle = robot.theta;
        // if (config.controlMethod == VorlifeControlMethod::EvolvedActiveVision || 
        //     config.controlMethod == VorlifeControlMethod::EvolvedActiveVisionPlusRandom)
        // {
            // Use the last two parameters to modify the sensor angle.
            double p1 = parameters.vec[parameters.vec.size() - 2];
            double p2 = parameters.vec[parameters.vec.size() - 1];
            sensorAngle += p1 * std::cos(alpha - p2 * M_PI);
        // }
        robot.sensorAngle = sensorAngle;

        Vec2 segmentStart = robot.pos + Vec2(cos(sensorAngle), sin(sensorAngle)) * (robot.radius + config.segmentSensorOffset);
        Vec2 segmentEnd = segmentStart + Vec2(cos(sensorAngle), sin(sensorAngle)) * config.segmentSensorLength;

        std::vector<CircleBody*> visiblePucks;
        if (config.useVoronoi)
            visiblePucks = getVoronoiVisiblePucks(worldState, robotIndexToDilatedPolygonsMap[robotIndex]);
        else
            visiblePucks = getAllPucks(worldState);

        // Find the closest puck that the segment intersects.
        bool hitPuck = false;
        double closestStartToPuckDistance = std::numeric_limits<double>::max();
        for (const CircleBody *puck : visiblePucks) {
            if (Utils::segmentIntersectsCircle(segmentStart, segmentEnd, puck->pos, puck->radius)) {
                hitPuck = true;
                double distance = (puck->pos - segmentStart).length() - puck->radius;
                if (distance < closestStartToPuckDistance)
                    closestStartToPuckDistance = distance;
            }
        }

        // Find the closest other robot that the segment intersects.
        bool hitRobot = false;
        double closestStartToRobotDistance = std::numeric_limits<double>::max();
        /*
        for (size_t otherRobotIndex = 0; otherRobotIndex < worldState->robots.size(); otherRobotIndex++) {
            if (otherRobotIndex == robotIndex)
                continue;

            Robot &otherRobot = worldState->robots[otherRobotIndex];
            if (Utils::segmentIntersectsCircle(segmentStart, segmentEnd, otherRobot.pos, otherRobot.radius)) {
                hitRobot = true;
                double distance = (otherRobot.pos - segmentStart).length() - otherRobot.radius;
                if (distance < closestStartToRobotDistance)
                    closestStartToRobotDistance = distance;
            }
        }
        */

        int hitValue = 0;
        if (hitPuck && (!hitRobot || closestStartToPuckDistance < closestStartToRobotDistance))
            hitValue = 1;
        else if (hitRobot && (!hitPuck || closestStartToRobotDistance < closestStartToPuckDistance))
            hitValue = 2;

        robotIndexToSensorReadings[robotIndex] = {segmentStart, segmentEnd, hitValue};
    }

    return robotIndexToSensorReadings;
}

}; // namespace VorlifeSensing