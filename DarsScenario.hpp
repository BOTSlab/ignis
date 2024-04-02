#pragma once
#include "Scenario.hpp"
#include "worldInitializer.hpp"
#include "Following.hpp"

class DarsScenario : public Scenario {
public:

    DarsScenario()
    {
        reset();

        // Perform a number of steps to resolve any initial collisions.
        for (int i=0; i<config.coldStartSteps; i++)
            Sim::update(simWorldState);
    }

    void update()
    {
        // Following::updateControlInputs(simWorldState, robotIndexToBestCurveMap);
    }

    void reset()
    {
        simWorldState = worldInitializer();
    }
};