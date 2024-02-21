#pragma once
#include <iostream>
#include <vector>

struct CircleBody {
    double x, y, radius;
    double vx, vy;

    CircleBody(double x, double y, double radius, double vx = 0.0, double vy = 0.0)
        : x(x), y(y), radius(radius), vx(vx), vy(vy) {}
};

struct Robot : public CircleBody {
    Robot(double x, double y, double radius, double theta, double vx = 0.0, double vy = 0.0)
        : CircleBody(x, y, radius, vx, vy), theta(theta) {}

    double theta;
};

struct WorldState {
    std::vector<Robot> robots;
    std::vector<CircleBody> pucks;

    void print() const {
        std::cout << "Robot Poses:" << std::endl;
        for (const auto& robot : robots) {
            std::cout << "x: " << robot.x << ", y: " << robot.y << ", theta: " << robot.theta << std::endl;
        }

        std::cout << "Puck Positions:" << std::endl;
        for (const auto& puck : pucks) {
            std::cout << "x: " << puck.x << ", y: " << puck.y << std::endl;
        }
    }
};