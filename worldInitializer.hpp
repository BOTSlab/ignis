#pragma once
#include <random>
#include <memory>
#include "WorldState.hpp"
#include "WorldConfig.hpp"

std::shared_ptr<WorldState> worldInitializer(const WorldConfig& config) {
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
        world->puckPoints.push_back({puckXDist(gen), puckYDist(gen)});
    for (int i = 0; i < config.numberOfRobots; i++)
        world->robotPoses.push_back({robotXDist(gen), robotYDist(gen), robotThetaDist(gen)});

    return world;
}