/**
 * Implements the experiment for my Forage 2024 paper.  Makes reference to
 * "Gauci et al." which is the following paper:
 * 
 * Gauci, M., Chen, J., Li, W., Dodd, T. J., & Groß, R. (2014). "Clustering
 * Objects with Robots That Do Not Compute". In Proceedings of the 2014
 * Conference on Autonomous Agents and Multi-Agent Systems (AAMAS 2014).
 * 
*/

#pragma once
#include "common/Scenario.hpp"
#include "common/Sim.hpp"
#include "common/WorldCreation.hpp"
#include "Forage/ForageSensing.hpp"
#include "Forage/ForageControl.hpp"

using namespace ForageSensing;

class ForageScenario : public Scenario {
public:
    ForageSensing::MapOfSensorReadings robotIndexToSensorReadingMap;

    double currentEvaluation = 0, cumulativeEvaluation = 0;

    ForageScenario()
    {
        reset();
    }

    void update()
    {
        robotIndexToSensorReadingMap = ForageSensing::allRobotsSense(simWorldState);
        ForageControl::allRobotsSetControls(simWorldState, robotIndexToSensorReadingMap);
        if (config.controlMethod == ForageControlMethod::EvolvedGauci)
            evaluateDispersion();
        else
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

    void evaluateDispersion()
    {
        currentEvaluation = simWorldState->getPuckDispersion();

        // Update the cumulative sum of dispersion * time, which corresponds
        // to equation (3) from Gauci et al.
        cumulativeEvaluation += currentEvaluation * stepCount;
    }

    void evaluateDistanceToGoal()
    {
        currentEvaluation = simWorldState->getSSDPucksToGoal();
        cumulativeEvaluation += currentEvaluation * stepCount;
    }
};