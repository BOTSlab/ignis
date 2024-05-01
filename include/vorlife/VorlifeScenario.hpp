/**
 * An experiment combining the minimalist controller submitted to ALIFE 2024
 * with Voronoi territories.
 * 
*/

#pragma once
#include <fstream>
#include "common/Scenario.hpp"
#include "common/Sim.hpp"
#include "common/WorldCreation.hpp"
#include "common/GeosVoronoi.hpp"
#include "vorlife/VorlifeSensing.hpp"
#include "vorlife/VorlifeControl.hpp"

using namespace VorlifeSensing;

class VorlifeScenario : public Scenario {
public:
    GeosVoronoi::GeosVoronoiBuilder voronoiBuilder;
    GeosVoronoi::MapOfVectorOfDilatedPolygons robotIndexToDilatedPolygonsMap;
    VorlifeSensing::MapOfSensorReadings robotIndexToSensorReadingMap;

    double currentEvaluation = 0, cumulativeEvaluation = 0;

    VorlifeScenario()
        : voronoiBuilder()
    {
        // Read in the parameters in last_parameters.dat into parameters.vec
        std::ifstream lastParametersFile("last_parameters.dat");
        if (lastParametersFile.is_open()) {
            parameters.vec.clear();
            double value;
            while (lastParametersFile >> value)
                parameters.vec.push_back(value);
            cout << "Read in the following parameters: ";
            for (int i = 0; i < parameters.vec.size(); ++i)
                cout << parameters.vec[i] << " ";
        } else {
            cout << "Could not open last_parameters.dat.  Using default parameters." << endl;
        }

        reset();
    }

    void update()
    {
        if (config.useVoronoi) {
            robotIndexToDilatedPolygonsMap.clear();
            voronoiBuilder.compute(simWorldState);
            voronoiBuilder.updateRobotSites(simWorldState);
            robotIndexToDilatedPolygonsMap = voronoiBuilder.getMapOfDilatedPolygons();
        }

        robotIndexToSensorReadingMap = VorlifeSensing::allRobotsSense(simWorldState, robotIndexToDilatedPolygonsMap);
        VorlifeControl::allRobotsSetControls(simWorldState, robotIndexToSensorReadingMap);
        
        evaluateDistanceToGoal();
    }

    void reset()
    {
        simWorldState = WorldCreation::randomWorld();
        // Perform a number of steps to resolve any initial collisions.
        for (int i=0; i<config.coldStartSteps; i++)
            Sim::update(simWorldState);

        stepCount = 0;
        currentEvaluation = 0;
        cumulativeEvaluation = 0;
    }

    void evaluateDistanceToGoal()
    {
        currentEvaluation = simWorldState->getSSDToGoal();
        cumulativeEvaluation += currentEvaluation * stepCount;
    }
};