#pragma once
#include "Scenario.hpp"
#include "WorldCreation.hpp"
#include "Sensing.hpp"
#include "Control.hpp"
#include "CurvesFromArcs.hpp"

class DarsScenario : public Scenario {
public:
    MapOfCurves robotIndexToCurveMap;
    MapOfSensorReadings robotIndexToSensorReadingsMap;

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

        robotIndexToSensorReadingsMap =  Sensing::allRobotsSenseTheirCurves(simWorldState, robotIndexToCurveMap);

        Control::allRobotsSetControls(simWorldState, robotIndexToSensorReadingsMap);
    }

    void reset()
    {
        simWorldState = WorldCreation::lineOfRobots();

        for (int i = 0; i < config.numberOfRobots; ++i) {
            std::vector<Curve> curves = CurvesFromArcs::arcCurvesForRobot(simWorldState->robots[i]);
            if (curves.size() == 0) {
                std::cerr << "No curves generated for robot " << i << std::endl;
                continue;
            }
            robotIndexToCurveMap[i] = curves[0];
        }
    }
};