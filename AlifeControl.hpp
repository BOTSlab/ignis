#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "WorldState.hpp"
#include "Config.hpp"
#include "CommonTypes.hpp"
//#include "AlifeSensingVersion1.hpp"
#include "AlifeSensingVersion2.hpp"
#include "Parameters.hpp"

using namespace CommonTypes;
//using namespace AlifeSensingVersion1;
using namespace AlifeSensingVersion2;

namespace AlifeControl {

/*
ControlInput evolvedGauci(const AlifeSensorReading &reading)
{
    if (reading.hit) {
        return {config.maxForwardSpeed, parameters.gauci[0] * config.maxAngularSpeed};
    } else {
        return {config.maxForwardSpeed, parameters.gauci[1] * config.maxAngularSpeed};
    }
}

ControlInput evolvedCubic(const AlifeSensorReading &reading)
{
    double a = reading.alpha;
    double angular = 0;
    if (reading.hit)
        angular = parameters.cubic[0] * config.maxAngularSpeed + 
                  parameters.cubic[1] * a * a * a + 
                  parameters.cubic[2] * a * a + 
                  parameters.cubic[3] * a;
    else
        angular = parameters.cubic[4] * config.maxAngularSpeed + 
                  parameters.cubic[5] * a * a * a + 
                  parameters.cubic[6] * a * a + 
                  parameters.cubic[7] * a;

    return {config.maxForwardSpeed, angular};        
}
*/

/*
ControlInput evolvedLinearVersion1(const AlifeSensorReadingVersion1 &reading)
{
    double a = reading.alpha;
    double angular = 0;
    if (reading.hitValue == 0)
        angular = parameters.linearVersion1[0] * config.maxAngularSpeed + 
                  parameters.linearVersion1[1] * a +
                  parameters.linearVersion1[2];

    else if (reading.hitValue == 1)
        angular = parameters.linearVersion1[3] * config.maxAngularSpeed + 
                  parameters.linearVersion1[4] * a +
                  parameters.linearVersion1[5];
    else if (reading.hitValue == 2)
        angular = parameters.linearVersion1[6] * config.maxAngularSpeed + 
                  parameters.linearVersion1[7] * a +
                  parameters.linearVersion1[8];
    else
        throw std::runtime_error("Unknown hit value");

    return {config.maxForwardSpeed, angular};        
}
*/

ControlInput evolvedLinearVersion2(const AlifeSensorReadingVersion2 &reading)
{
    double a = reading.alpha;
    double angular = 0;
    if (!reading.hitPuck && !reading.hitRobot)
        angular = parameters.linearVersion2[0] * config.maxAngularSpeed + 
                  parameters.linearVersion2[1] * a +
                  parameters.linearVersion2[2];
    else if (reading.hitPuck && !reading.hitRobot)
        angular = parameters.linearVersion2[3] * config.maxAngularSpeed + 
                  parameters.linearVersion2[4] * a +
                  parameters.linearVersion2[5];
    else if (!reading.hitPuck && reading.hitRobot)
        angular = parameters.linearVersion2[6] * config.maxAngularSpeed + 
                  parameters.linearVersion2[7] * a +
                  parameters.linearVersion2[8];
    else if (reading.hitPuck && reading.hitRobot)
        angular = parameters.linearVersion2[9] * config.maxAngularSpeed + 
                  parameters.linearVersion2[10] * a +
                  parameters.linearVersion2[11];
    else
        throw std::runtime_error("Unknown hit value");

    return {config.maxForwardSpeed, angular};        
}

void allRobotsSetControls(std::shared_ptr<WorldState> worldState, MapOfSensorReadingsVersion2 &robotIndexToSensorReadings)
{
    for (const auto &robotIndexAndSensorReading : robotIndexToSensorReadings)
    {
        size_t robotIndex = robotIndexAndSensorReading.first;
        Robot &robot = worldState->robots[robotIndex];
        const auto &sensorReading = robotIndexAndSensorReading.second;

/*        if (config.controlMethod == AlifeControlMethod::EvolvedGauci)
            robot.controlInput = evolvedGauci(sensorReading);
        else if (config.controlMethod == AlifeControlMethod::EvolvedCubic)
            robot.controlInput = evolvedCubic(sensorReading);
        else */if (config.controlMethod == AlifeControlMethod::EvolvedLinearVersion2)
            robot.controlInput = evolvedLinearVersion2(sensorReading);
        else
            throw std::runtime_error("Unknown control method");
    }
}

}; // namespace AlifeControl