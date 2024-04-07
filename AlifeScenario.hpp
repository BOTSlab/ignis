#pragma once
#include "Scenario.hpp"
#include "WorldCreation.hpp"
#include "AlifeSensing.hpp"
#include "AlifeControl.hpp"

using namespace AlifeSensing;

class AlifeScenario : public Scenario {
public:
    MapOfSensorReadings robotIndexToSensorReadingMap;

    AlifeScenario()
    {
        reset();

        // Perform a number of steps to resolve any initial collisions.
        for (int i=0; i<config.coldStartSteps; i++)
            Sim::update(simWorldState);
    }

    void update()
    {
        robotIndexToSensorReadingMap = AlifeSensing::allRobotsSense(simWorldState);
        AlifeControl::allRobotsSetControls(simWorldState, robotIndexToSensorReadingMap);
    }

    void reset()
    {
        simWorldState = WorldCreation::randomWorld();
    }
};