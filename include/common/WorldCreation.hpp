#pragma once
#include <random>
#include <memory>
#include "common/WorldState.hpp"
#include "common/Config.hpp"

namespace WorldCreation {

std::shared_ptr<WorldState> randomWorld() {
    // Create a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> puckXDist(config.puckRadius, config.width - config.puckRadius);
    std::uniform_real_distribution<double> puckYDist(config.puckRadius, config.height - config.puckRadius);
    std::uniform_real_distribution<double> robotXDist(config.robotRadius, config.width - config.robotRadius);
    std::uniform_real_distribution<double> robotYDist(config.robotRadius, config.height - config.robotRadius);
    std::uniform_real_distribution<double> robotThetaDist(-M_PI, M_PI);
    std::uniform_real_distribution<double> goalXDist(0, config.width);
    std::uniform_real_distribution<double> goalYDist(0, config.height);

    // Create an instance of WorldState using make_shared
    auto world = std::make_shared<WorldState>();

    // Generate randomly position pucks and robots.
    for (int i = 0; i < config.numberOfPucks; i++)
        world->pucks.push_back(CircleBody(BodyType::Puck, puckXDist(gen), puckYDist(gen), config.puckRadius, 1.0));
    for (int i = 0; i < config.numberOfRobots; i++)
        world->robots.push_back(Robot(robotXDist(gen), robotYDist(gen), config.robotRadius, 100.0,  robotThetaDist(gen)));

    // Generate a random goal position
    world->goalPos = Vec2(goalXDist(gen), goalYDist(gen));

    return world;
}

std::shared_ptr<WorldState> lineOfRobots() {
    auto world = std::make_shared<WorldState>();

    // Create a line of robots
    for (int i = 0; i < config.numberOfRobots; i++)
        world->robots.push_back(Robot(100 + i * 200, 300, config.robotRadius, 100.0, 0));

    world->goalPos = Vec2(config.width/2, config.height/2);

    return world;
}

}; // namespace WorldCreation