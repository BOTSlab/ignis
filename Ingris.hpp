#pragma once
#include "WorldConfig.hpp"
#include "worldInitializer.hpp"

class Ingris {
    public:
        WorldConfig config;
        std::shared_ptr<WorldState> simWorldState;

        Ingris() {
            simWorldState = worldInitializer(config);
        }

        void runSim() {
            // Run the simulation
        }
};