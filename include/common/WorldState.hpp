#pragma once
#include <iostream>
#include <vector>
#include "common/CommonTypes.hpp"
#include "common/Config.hpp"

using namespace CommonTypes;

struct WorldState
{
    std::vector<Robot> robots;
    std::vector<CircleBody> pucks;
    int nRobotRobotCollisions = 0;
    int nRobotPuckCollisions = 0;
    int nRobotBoundaryCollisions = 0;

    Vec2 goalPos;

    void resetCollisionCounts()
    {
        nRobotRobotCollisions = 0;
        nRobotPuckCollisions = 0;
        nRobotBoundaryCollisions = 0;
    }

    // BAD: Is there a better way to get all the circle bodies?
    std::vector<CircleBody *> getAllCircleBodies()
    {
        std::vector<CircleBody *> allBodies;
        for (const auto &robot : robots)
            allBodies.push_back((CircleBody *)&robot);
        for (const auto &puck : pucks)
            allBodies.push_back((CircleBody *)&puck);
        return allBodies;
    }

    void print() const
    {
        std::cout << "Robot Poses:" << std::endl;
        for (const auto &robot : robots)
        {
            std::cout << "x: " << robot.pos.x << ", y: " << robot.pos.y << ", theta: " << robot.theta << std::endl;
        }

        std::cout << "Puck Positions:" << std::endl;
        for (const auto &puck : pucks)
        {
            std::cout << "x: " << puck.pos.x << ", y: " << puck.pos.y << std::endl;
        }
    }

    double getPuckDispersion() const
    {
        // Compute the centroid of puck positions.
        Vec2 sum(0.0, 0.0);
        for (const auto& puck : pucks)
            sum += puck.pos;
        Vec2 centroid(0.0, 0.0);
        double nPucks = pucks.size();
        if (pucks.size() > 0) {
            centroid.x = sum.x / nPucks;
            centroid.y = sum.y / nPucks;
        }

        // Now compute dispersion, the second moment of puck positions, which
        // corresponds to equation (2) from Gauci et al.  Effectively, it is
        // a scaled version of sum of squared distance of each puck from the centroid.
        double sumSquaredDistances = 0.0;
        for (const auto& puck : pucks)
            sumSquaredDistances += (puck.pos - centroid).lengthSquared();

        return sumSquaredDistances / (4 * config.puckRadius * config.puckRadius);
    }

    double getSSDPucksToGoal() const
    {
        double sumSquaredDistances = 0.0;
        for (const auto& puck : pucks)
            sumSquaredDistances += (puck.pos - goalPos).lengthSquared();
        return sumSquaredDistances;
    }

    double getAverageRobotRobotDistance() const
    {
        double sumDistances = 0.0;
        for (size_t i = 0; i < robots.size(); ++i)
        {
            for (size_t j = i + 1; j < robots.size(); ++j)
            {
                sumDistances += (robots[i].pos - robots[j].pos).length();
            }
        }
        return sumDistances / (robots.size() * (robots.size() - 1) / 2);
    }

    double getAverageRobotAngularSpeed() const
    {
        double sumAngularSpeeds = 0.0;
        for (const auto& robot : robots)
            sumAngularSpeeds += robot.controlInput.angularSpeed;
        return sumAngularSpeeds / robots.size();
    }
};