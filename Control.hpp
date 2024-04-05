#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "WorldState.hpp"
#include "WorldConfig.hpp"
#include "CommonTypes.hpp"

using namespace CommonTypes;

namespace Control {

void setControls(Robot &robot, const std::vector<SensorReading> &readings)
{
    double l = readings[0].value;
    double cl = readings[1].value;
    double cr = readings[2].value;
    double r = readings[3].value;

    if (l == 0 && cl == 0 && cr == 0 && r == 0)
    {
        robot.controlInput = {0, 0};
        return;
    }

    double balanceFactor = -l - cl + cr + r;
    robot.controlInput = {config.maxForwardSpeed, 0.05 * config.maxAngularSpeed * balanceFactor};
}

void allRobotsSetControls(std::shared_ptr<WorldState> worldState, MapOfSensorReadings &robotIndexToSensorReadings)
{
    for (const auto &robotIndexAndSensorReadings : robotIndexToSensorReadings)
    {
        size_t robotIndex = robotIndexAndSensorReadings.first;
        const std::vector<SensorReading> &sensorReadings = robotIndexAndSensorReadings.second;

        setControls(worldState->robots[robotIndex], sensorReadings);
    }
}

}; // namespace Control