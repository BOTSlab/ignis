#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "common/WorldState.hpp"
#include "common/Config.hpp"
#include "common/CommonTypes.hpp"
#include "vorlife/VorlifeSensing.hpp"
#include "forage/Parameters.hpp"

using namespace CommonTypes;
using namespace VorlifeSensing;

namespace VorlifeControl {

// This is the controller for the Gauci et al. evolved controller (EvolvedGauci)
// as well as the evolved active vision controller (EvolvedActiveVision).
ControlInput gauciControl(const VorlifeSensorReading &reading)
{
    double angular = 0;
    if (reading.hitValue == 0)
        angular = parameters.vec[0];

    else if (reading.hitValue == 1)
        angular = parameters.vec[1];

    else if (reading.hitValue == 2)
        angular = parameters.vec[2];

    else
        throw std::runtime_error("Unknown hit value");

    return {config.maxForwardSpeed, angular * config.maxAngularSpeed};        
}

void allRobotsSetControls(std::shared_ptr<WorldState> worldState, VorlifeSensing::MapOfSensorReadings &robotIndexToSensorReadings)
{
    for (const auto &robotIndexAndSensorReading : robotIndexToSensorReadings)
    {
        size_t robotIndex = robotIndexAndSensorReading.first;
        Robot &robot = worldState->robots[robotIndex];
        const auto &sensorReading = robotIndexAndSensorReading.second;

        // if (config.controlMethod == VorlifeControlMethod::EvolvedGauci || 
        //     config.controlMethod == VorlifeControlMethod::EvolvedActiveVision)
            robot.controlInput = gauciControl(sensorReading);
        // else if (config.controlMethod == VorlifeControlMethod::EvolvedActiveVisionPlusRandom)
        //     robot.controlInput = gauciControlPlusRandom(sensorReading);
        // else
        //     throw std::runtime_error("Unknown control method!");
    }
}

}; // namespace VorlifeControl