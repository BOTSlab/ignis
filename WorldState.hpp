#pragma once
#include <iostream>
#include <vector>

struct Point {
    double x;
    double y;
};

struct Pose {
    double x;
    double y;
    double theta;
};

struct WorldState {
    std::vector<Pose> robotPoses;
    std::vector<Point> puckPoints;

    void print() const {
        std::cout << "Robot Poses:" << std::endl;
        for (const auto& pose : robotPoses) {
            std::cout << "x: " << pose.x << ", y: " << pose.y << ", theta: " << pose.theta << std::endl;
        }

        std::cout << "Puck Points:" << std::endl;
        for (const auto& point : puckPoints) {
            std::cout << "x: " << point.x << ", y: " << point.y << std::endl;
        }
    }
};