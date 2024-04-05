#pragma once
#include <vector>
#include <map>
#include <cmath>
#include <utility>
#include "WorldState.hpp"
#include "WorldConfig.hpp"
#include "CommonTypes.hpp"

using namespace CommonTypes;

namespace Sensing {

// Compute the distance from point x to the line segment defined by points p1 and p2.
double distToSegment(const Vec2& x, const Vec2& p1, const Vec2& p2)
{
    Vec2 v = p2 - p1;
    Vec2 w = x - p1;

    double c1 = w.dot(v);
    if (c1 <= 0)
        return w.length();

    double c2 = v.dot(v);
    if (c2 <= c1)
        return (x - p2).length();

    double b = c1 / c2;
    Vec2 Pb = p1 + v * b;
    return (x - Pb).length();
}

std::vector<SensorReading> senseCurve(const Robot &robot, const Curve &curve)
{
    std::vector<SensorReading> sensorReadings;

    // For each sensor, calculate the distance to the nearest point on the curve.
    for (const SensorPosition &relativePos : config.sensorPositions)
    {
        // Get the absolute position of the sensor
        Vec2 pos = Vec2(robot.x, robot.y) + Vec2(relativePos.forwardOffset, relativePos.lateralOffset).rotate(robot.theta);

        // Determine the two adjacent points on the curve that are closest to the sensor.
        double minDist = std::numeric_limits<double>::max();
        Vec2 closestPoint1, closestPoint2;
        for (size_t i = 0; i < curve.points.size() - 1; i++)
        {
            Vec2 p1 = Vec2(curve.points[i].pose.x, curve.points[i].pose.y);
            Vec2 p2 = Vec2(curve.points[i + 1].pose.x, curve.points[i + 1].pose.y);

            // Calculate the distance from the sensor to the line segment defined by p1 and p2
            double dist = distToSegment(pos, p1, p2);
            if (dist < minDist)
            {
                minDist = dist;
                closestPoint1 = p1;
                closestPoint2 = p2;
            }
        }

        if (minDist < config.curveThickness / 2)
            sensorReadings.push_back(SensorReading{relativePos, 1});
        else
            sensorReadings.push_back(SensorReading{relativePos, 0});

    }

    return sensorReadings;
}

MapOfSensorReadings allRobotsSenseTheirCurves(std::shared_ptr<WorldState> worldState, MapOfCurves &robotIndexToCurve)
{
    MapOfSensorReadings robotIndexToSensorReadings;

    for (const auto &pair : robotIndexToCurve)
    {
        size_t robotIndex = pair.first;
        const Curve &curve = pair.second;
        std::vector<SensorReading> sensorReadings = senseCurve(worldState->robots[robotIndex], curve);
        robotIndexToSensorReadings[robotIndex] = sensorReadings;
    }

    return robotIndexToSensorReadings;
}

}; // namespace Sensing