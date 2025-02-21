#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "common/WorldState.hpp"
#include "common/Config.hpp"
#include "common/CommonTypes.hpp"
#include "forage/ForageSensing.hpp"
#include "forage/Parameters.hpp"

using namespace CommonTypes;
using namespace ForageSensing;

namespace ForageControl {

// This is the controller for the Gauci et al. evolved controller (EvolvedGauci)
// as well as the evolved active vision controller (EvolvedActiveVision).
ControlInput gauciControl(const ForageSensorReading &reading)
{
    double angular = 0;
    if (reading.hitValue == 0)
        angular = parameters.vec[0];

    else if (reading.hitValue == 1)
        angular = parameters.vec[1];

    else if (reading.hitValue == 2)
        angular = parameters.vec[2];

    else
        throw std::runtime_error("Unknown hit value: " + std::to_string(reading.hitValue));

    return {config.maxForwardSpeed, angular * config.maxAngularSpeed};        
}

// This is the controller for the evolved active vision controller with random 
// noise added (EvolvedActiveVisionPlusRandom).
ControlInput gauciControlPlusRandom(const ForageSensorReading &reading)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_real_distribution<double> rand(-1.0, 1.0);

    double angular = 0;
    if (reading.hitValue == 0)
        angular = parameters.vec[0] + parameters.vec[1] * rand(gen);

    else if (reading.hitValue == 1)
        angular = parameters.vec[2] + parameters.vec[3] * rand(gen);

    else if (reading.hitValue == 2)
        angular = parameters.vec[4] + parameters.vec[5] * rand(gen);

    else
        throw std::runtime_error("Unknown hit value");

    return {config.maxForwardSpeed, angular * config.maxAngularSpeed};        
}

/*
ControlInput evolvedCubic(const ForageSensorReading &reading)
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
ControlInput evolvedLinear(const ForageSensorReading &reading)
{
    double a = reading.alpha;
    double angular = 0;
    if (reading.hitValue == 0)
        angular = parameters.vec[0] * config.maxAngularSpeed + 
                  parameters.vec[1] * a +
                  parameters.vec[2];

    else if (reading.hitValue == 1)
        angular = parameters.vec[3] * config.maxAngularSpeed + 
                  parameters.vec[4] * a +
                  parameters.vec[5];
    else if (reading.hitValue == 2)
        angular = parameters.vec[6] * config.maxAngularSpeed + 
                  parameters.vec[7] * a +
                  parameters.vec[8];
    else
        throw std::runtime_error("Unknown hit value");

    return {config.maxForwardSpeed, angular};        
}
*/

/* spinner3 
ControlInput evolvedSpinner(const ForageSensorReading &reading)
{
    double alpha = reading.alpha;

    // We will assume that all parameters below are in the range [-1, 1].
    double deviation = 0;
    if (reading.hitValue == 0)
        deviation = parameters.spinner[0] * alpha / M_PI;

    else if (reading.hitValue == 1)
        deviation = parameters.spinner[1] * alpha / M_PI;
        
    else if (reading.hitValue == 2)
        deviation = parameters.spinner[2] * alpha / M_PI;
        
    else
        throw std::runtime_error("Unknown hit value");

    double angular = config.nominalSpin + Utils::scale(deviation, -1, 1, -config.maxDeviation, config.maxDeviation);

    return {config.maxForwardSpeed, angular * config.maxAngularSpeed};        
}
*/

/*
ControlInput evolvedSpinner6(const ForageSensorReading &reading)
{
    double alpha = reading.alpha;

    // We will assume that all parameters below are in the range [-1, 1].
    double angular = 0;
    if (reading.hitValue == 0)
        angular = parameters.vec[0] * std::cos(alpha - parameters.vec[1] * M_PI);

    else if (reading.hitValue == 1)
        angular = parameters.vec[2] * std::cos(alpha - parameters.vec[3] * M_PI);
        
    else if (reading.hitValue == 2)
        angular = parameters.vec[4] * std::cos(alpha - parameters.vec[5] * M_PI);
        
    else
        throw std::runtime_error("Unknown hit value");

    return {config.maxForwardSpeed, angular * config.maxAngularSpeed};        
}
*/

void allRobotsSetControls(std::shared_ptr<WorldState> worldState, ForageSensing::MapOfSensorReadings &robotIndexToSensorReadings)
{
    for (const auto &robotIndexAndSensorReading : robotIndexToSensorReadings)
    {
        size_t robotIndex = robotIndexAndSensorReading.first;
        Robot &robot = worldState->robots[robotIndex];
        const auto &sensorReading = robotIndexAndSensorReading.second;

        if (config.controlMethod == ForageControlMethod::EvolvedGauci || 
            config.controlMethod == ForageControlMethod::EvolvedActiveVision)
            robot.controlInput = gauciControl(sensorReading);
        else if (config.controlMethod == ForageControlMethod::EvolvedActiveVisionPlusRandom)
            robot.controlInput = gauciControlPlusRandom(sensorReading);
        else
            throw std::runtime_error("Unknown control method!");
    }
}

}; // namespace ForageControl