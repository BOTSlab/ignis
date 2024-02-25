#pragma once
#include <iostream>
#include <vector>

struct CircleBody {
    double x, y, radius, mass;
    double vx, vy;
    int nCollisions = 0;

    CircleBody(double x, double y, double radius, double mass, double vx = 0.0, double vy = 0.0)
        : x(x), y(y), radius(radius), mass(mass), vx(vx), vy(vy) {}
};

struct ControlInput {
    double forwardSpeed, angularSpeed;

    ControlInput(double forwardSpeed = 0.0, double angularSpeed = 0.0)
        : forwardSpeed(forwardSpeed), angularSpeed(angularSpeed) {}
};

struct Robot : public CircleBody {
    Robot(double x, double y, double radius, double mass, double theta, double vx = 0.0, double vy = 0.0)
        : CircleBody(x, y, radius, mass, vx, vy), theta(theta) {}

    double theta;
    ControlInput controlInput;
};

struct WorldState {
    std::vector<Robot> robots;
    std::vector<CircleBody> pucks;

    // BAD: Is there a better way to get all the circle bodies?
    std::vector<CircleBody*> getAllCircleBodies() {
        std::vector<CircleBody*> allBodies;
        for (const auto& robot : robots)
            allBodies.push_back((CircleBody*) &robot);
        for (const auto& puck : pucks)
            allBodies.push_back((CircleBody*) &puck);
        return allBodies;
    }

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