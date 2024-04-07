#pragma once
#include <iostream>
#include <vector>
#include "CommonTypes.hpp"

using namespace CommonTypes;

struct WorldState
{
    std::vector<Robot> robots;
    std::vector<CircleBody> pucks;
    int nRobotRobotCollisions = 0;
    int nRobotPuckCollisions = 0;
    int nRobotBoundaryCollisions = 0;

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
};