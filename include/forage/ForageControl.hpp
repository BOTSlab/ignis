#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "common/WorldState.hpp"
#include "common/Config.hpp"
#include "common/CommonTypes.hpp"
#include "forage/ForageSensing.hpp"

using namespace CommonTypes;
using namespace ForageSensing;

namespace ForageControl {

// This is the controller for the Gauci et al. evolved controller (EvolvedGauci)
// as well as the evolved active vision controller (EvolvedActiveVision).
ControlInput gauciControl(const ForageSensorReading &reading)
{
    Config &config = Config::getInstance();

    double angular = 0;
    if (reading.hitValue == 0)
        angular = config.controlParameters[0];

    else if (reading.hitValue == 1)
        angular = config.controlParameters[1];

    else if (reading.hitValue == 2)
        angular = config.controlParameters[2];

    else
        throw std::runtime_error("Unknown hit value: " + std::to_string(reading.hitValue));

    return {config.maxForwardSpeed, angular * config.maxAngularSpeed};        
}

void allRobotsSetControls(std::shared_ptr<WorldState> worldState, ForageSensing::MapOfSensorReadings &robotIndexToSensorReadings)
{
    Config &config = Config::getInstance();

    for (const auto &robotIndexAndSensorReading : robotIndexToSensorReadings)
    {
        size_t robotIndex = robotIndexAndSensorReading.first;
        Robot &robot = worldState->robots[robotIndex];
        const auto &sensorReading = robotIndexAndSensorReading.second;

        if (config.controlMethod == ControlMethod::ThreeParameterCluster || 
            config.controlMethod == ControlMethod::FiveParameterForage)
            robot.controlInput = gauciControl(sensorReading);
        else
            throw std::runtime_error("Unknown control method!");
    }
}

}; // namespace ForageControl