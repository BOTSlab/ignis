#pragma once
#include "WorldConfig.hpp"
#include "worldInitializer.hpp"
#include "Sim.hpp"
#include "Planning.hpp"
#include "CurveFollowing.hpp"

class Ignis {
    public:
        WorldConfig config;

        std::shared_ptr<WorldState> simWorldState;

        std::shared_ptr<Plan> plan;

        Ignis() : config() {
            reset();
        }

        void step() {
            Sim::update(simWorldState);
            plan = Planning::plan(simWorldState);
            CurveFollowing::updateControlInputs(simWorldState, plan);
        }

        void reset() {
            simWorldState = worldInitializer();
        }
};