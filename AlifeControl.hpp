#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "WorldState.hpp"
#include "Config.hpp"
#include "CommonTypes.hpp"
#include "AlifeSensing.hpp"
#include "Parameters.hpp"

using namespace CommonTypes;
using namespace AlifeSensing;

namespace AlifeControl {

ControlInput hardCodedAlife(const AlifeSensorReading &reading)
{
// std::cerr << reading.alpha << std::endl;
    if (reading.hit) {
        return {config.maxForwardSpeed, 0.1*config.maxAngularSpeed + 0.1*reading.alpha};
    } else {
        return {config.maxForwardSpeed, -0.1*config.maxAngularSpeed - 0.1*reading.alpha};
    }
}

ControlInput hardCodedGauci(const AlifeSensorReading &reading)
{
    if (reading.hit) {
        return {config.maxForwardSpeed, parameters.gauci[0] * config.maxAngularSpeed};
    } else {
        return {config.maxForwardSpeed, parameters.gauci[1] * config.maxAngularSpeed};
    }
}

void allRobotsSetControls(std::shared_ptr<WorldState> worldState, MapOfSensorReadings &robotIndexToSensorReadings)
{
    for (const auto &robotIndexAndSensorReading : robotIndexToSensorReadings)
    {
        size_t robotIndex = robotIndexAndSensorReading.first;
        Robot &robot = worldState->robots[robotIndex];
        const AlifeSensorReading &sensorReading = robotIndexAndSensorReading.second;

        robot.controlInput = hardCodedGauci(sensorReading);
        //robot.controlInput = hardCodedAlife(sensorReading);
    }
}

}; // namespace AlifeControl