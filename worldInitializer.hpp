#pragma once
#include <random>
#include <memory>
#include "WorldState.hpp"
#include "WorldConfig.hpp"

std::shared_ptr<WorldState> worldInitializer() {
    // Create a random number generator
    std::random_device rd;
    std::mt19937 gen(rd());

    std::uniform_real_distribution<double> puckXDist(config.puckRadius, config.width - config.puckRadius);
    std::uniform_real_distribution<double> puckYDist(config.puckRadius, config.height - config.puckRadius);
    std::uniform_real_distribution<double> robotXDist(config.robotRadius, config.width - config.robotRadius);
    std::uniform_real_distribution<double> robotYDist(config.robotRadius, config.height - config.robotRadius);
    std::uniform_real_distribution<double> robotThetaDist(-M_PI, M_PI);

    // Create an instance of WorldState using make_shared
    auto world = std::make_shared<WorldState>();

    // Generate randomly position pucks and robots.
    for (int i = 0; i < config.numberOfPucks; i++)
        world->pucks.push_back(CircleBody(puckXDist(gen), puckYDist(gen), config.puckRadius, 1.0));
    for (int i = 0; i < config.numberOfRobots; i++)
        world->robots.push_back(Robot(robotXDist(gen), robotYDist(gen), config.robotRadius, 100.0,  robotThetaDist(gen)));

    return world;
}