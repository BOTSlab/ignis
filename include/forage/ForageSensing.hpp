#pragma once
#include "common/Angles.hpp"
#include "common/CommonTypes.hpp"
#include "common/Config.hpp"
#include "common/Utils.hpp"
#include "common/WorldState.hpp"
#include "forage/Parameters.hpp"
#include <cmath>
#include <map>
#include <utility>
#include <vector>

using namespace CommonTypes;

namespace ForageSensing {

//struct ForageSensorReading {
//    Vec2 segmentStart, segmentEnd;
//    int hitValue; // 0: no hit, 1: hit puck, 2: hit robot
//};

struct OneSensorReading {
    Vec2 segmentStart, segmentEnd;
    bool hit;
};

struct ForageSensorReading {
    OneSensorReading puckReading;
    OneSensorReading robotReading;
};

// A map from robot index to it's sensor reading.
using MapOfSensorReadings = std::map<size_t, ForageSensorReading>;

// This function is used to sense both pucks and robots along a segment.
// This should be used for compatibility with Gauci et al, but I'm now
// experimenting with separating the two with the two functions that follow.
/*
ForageSensorReading senseBothPucksAndRobotsAlongSegment(
    shared_ptr<WorldState> &worldState, 
    CommonTypes::Robot &robot,
    size_t &robotIndex)
{
    // Determine alpha, the angle between the robot's heading and the local
    // goal direction.
    double angleToGoal = atan2(worldState->goalPos.y - robot.pos.y,
                               worldState->goalPos.x - robot.pos.x);
    double alpha =
        Angles::getSmallestSignedAngularDifference(angleToGoal, robot.theta);

    // This is the active vision component, which can modify the sensor's angle.
    double sensorAngle = robot.theta;
    if (config.controlMethod == ForageControlMethod::EvolvedActiveVision ||
        config.controlMethod ==
            ForageControlMethod::EvolvedActiveVisionPlusRandom) {
        // Use parameters K_3 and K_4 to modify the sensor angle.
        double p1 = parameters.vec[3];
        double p2 = parameters.vec[4];
        sensorAngle += p1 * std::cos(alpha - p2 * M_PI);
        // cerr << "alpha: " << alpha << " sensorAngle: " << sensorAngle <<
        // endl;
    }
    robot.sensorAngle = sensorAngle;

    Vec2 segmentStart =
        robot.pos + Vec2(cos(sensorAngle), sin(sensorAngle)) *
                        (robot.radius + config.segmentSensorOffset);
    Vec2 segmentEnd = segmentStart + Vec2(cos(sensorAngle), sin(sensorAngle)) *
                                         parameters.vec[5] *
                                         config.maxSegmentSensorLength;

    // Find the closest puck that the segment intersects.
    bool hitPuck = false;
    double closestStartToPuckDistance = std::numeric_limits<double>::max();
    for (const CircleBody &puck : worldState->pucks) {
        if (Utils::segmentIntersectsCircle(segmentStart, segmentEnd, puck.pos,
                                           puck.radius)) {

            hitPuck = true;
            double distance = (puck.pos - segmentStart).length() - puck.radius;
            if (distance < closestStartToPuckDistance)
                closestStartToPuckDistance = distance;
        }
    }

    // Find the closest other robot that the segment intersects.
    bool hitRobot = false;
    double closestStartToRobotDistance = std::numeric_limits<double>::max();
    for (size_t otherRobotIndex = 0;
         otherRobotIndex < worldState->robots.size(); otherRobotIndex++) {
        if (otherRobotIndex == robotIndex)
            continue;

        Robot &otherRobot = worldState->robots[otherRobotIndex];
        if (Utils::segmentIntersectsCircle(segmentStart, segmentEnd,
                                           otherRobot.pos, otherRobot.radius)) {
            hitRobot = true;
            double distance =
                (otherRobot.pos - segmentStart).length() - otherRobot.radius;
            if (distance < closestStartToRobotDistance)
                closestStartToRobotDistance = distance;
        }
    }

    int hitValue = 0;
    if (hitPuck &&
        (!hitRobot || closestStartToPuckDistance < closestStartToRobotDistance))
        hitValue = 1;
    else if (hitRobot && (!hitPuck || closestStartToRobotDistance <
                                          closestStartToPuckDistance))
        hitValue = 2;
    
    return {segmentStart, segmentEnd, hitValue};
}
*/

OneSensorReading senseJustPucksAlongSegment(
    shared_ptr<WorldState> &worldState, 
    CommonTypes::Robot &robot,
    size_t &robotIndex)
{
    // Determine alpha, the angle between the robot's heading and the local
    // goal direction.
    double angleToGoal = atan2(worldState->goalPos.y - robot.pos.y,
                               worldState->goalPos.x - robot.pos.x);
    double alpha =
        Angles::getSmallestSignedAngularDifference(angleToGoal, robot.theta);

    // This is the active vision component, which can modify the sensor's angle.
    double sensorAngle = robot.theta;
    if (config.controlMethod == ForageControlMethod::EvolvedActiveVision ||
        config.controlMethod ==
            ForageControlMethod::EvolvedActiveVisionPlusRandom) {
        // Use parameters K_3 and K_4 to modify the sensor angle.
        double p1 = parameters.vec[3];
        double p2 = parameters.vec[4];
        sensorAngle += p1 * std::cos(alpha - p2 * M_PI);
        // cerr << "alpha: " << alpha << " sensorAngle: " << sensorAngle <<
        // endl;
    }
    //robot.sensorAngle = sensorAngle;

    Vec2 segmentStart =
        robot.pos + Vec2(cos(sensorAngle), sin(sensorAngle)) *
                        (robot.radius + config.segmentSensorOffset);
    Vec2 segmentEnd = segmentStart + Vec2(cos(sensorAngle), sin(sensorAngle)) *
                                         parameters.vec[5] *
                                         config.maxSegmentSensorLength;

    // Check if the segment intersects any puck.
    bool hitPuck = false;
    double closestStartToPuckDistance = std::numeric_limits<double>::max();
    for (const CircleBody &puck : worldState->pucks) {
        if (Utils::segmentIntersectsCircle(segmentStart, segmentEnd, puck.pos,
                                           puck.radius)) {

            hitPuck = true;
            break;
        }
    }
    
    return {segmentStart, segmentEnd, hitPuck};
}

OneSensorReading senseJustRobotsAlongSegment(
    shared_ptr<WorldState> &worldState, 
    CommonTypes::Robot &robot,
    size_t &robotIndex)
{
    double sensorAngle = robot.theta;
    //robot.sensorAngle = sensorAngle;

    Vec2 segmentStart =
        robot.pos + Vec2(cos(sensorAngle), sin(sensorAngle)) *
                        (robot.radius + config.segmentSensorOffset);
    Vec2 segmentEnd = segmentStart + Vec2(cos(sensorAngle), sin(sensorAngle)) *
                                         parameters.vec[6] *
                                         config.maxSegmentSensorLength;

    // Check if the fattened segment intersects any other robots.
    bool hitRobot = false;
    for (size_t otherRobotIndex = 0;
         otherRobotIndex < worldState->robots.size(); otherRobotIndex++) {
        if (otherRobotIndex == robotIndex)
            continue;

        Robot &otherRobot = worldState->robots[otherRobotIndex];

        // We are "fattening the segment" by dividing the robot's radius by
        // parameters.vec[7].
        if (Utils::segmentIntersectsCircle(segmentStart, segmentEnd,
                                           otherRobot.pos, otherRobot.radius / parameters.vec[7])) {
            hitRobot = true;
            break;
        }
    }
    
    return {segmentStart, segmentEnd, hitRobot};
}

/**
 * Gets sensor readings for all robots in the world.
 *
 * @param worldState A shared pointer to the current world state.
 * @return A map of robot indices to their corresponding sensor readings.
 */
MapOfSensorReadings allRobotsSense(std::shared_ptr<WorldState> worldState) {
    MapOfSensorReadings robotIndexToSensorReadings;

    // Loop through all robots and get their sensor readings.
    for (size_t robotIndex = 0; robotIndex < worldState->robots.size();
         robotIndex++) {
        Robot &robot = worldState->robots[robotIndex];

        if (robot.slowedCounter > 0) {
            // A hack to speed up the simulation.  Slowed robots don't sense
            // anything.
            robotIndexToSensorReadings[robotIndex] = {0, 0};
            continue;
        }

        //robotIndexToSensorReadings[robotIndex] = senseBothPucksAndRobotsAlongSegment(worldState, robot, robotIndex);
        ForageSensorReading sensorReading;
        sensorReading.puckReading = senseJustPucksAlongSegment(worldState, robot, robotIndex);
        sensorReading.robotReading = senseJustRobotsAlongSegment(worldState, robot, robotIndex);
        robotIndexToSensorReadings[robotIndex] = sensorReading;
    }

    return robotIndexToSensorReadings;
}

}; // namespace ForageSensing