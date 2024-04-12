/**
 * Implements the experiment for my ALIFE 2024 paper.  Makes reference to
 * "Gauci et al." which is the following paper:
 * 
 * Gauci, M., Chen, J., Li, W., Dodd, T. J., & Groß, R. (2014). "Clustering
 * Objects with Robots That Do Not Compute". In Proceedings of the 2014
 * Conference on Autonomous Agents and Multi-Agent Systems (AAMAS 2014).
 * 
*/

#pragma once
#include "Scenario.hpp"
#include "WorldCreation.hpp"
#include "AlifeSensing.hpp"
#include "AlifeControl.hpp"

using namespace AlifeSensing;

class AlifeScenario : public Scenario {
public:
    MapOfSensorReadings robotIndexToSensorReadingMap;

    double currentEvaluation = 0, cumulativeEvaluation = 0;

    AlifeScenario()
    {
        reset();
    }

    void update()
    {
        robotIndexToSensorReadingMap = AlifeSensing::allRobotsSense(simWorldState);
        AlifeControl::allRobotsSetControls(simWorldState, robotIndexToSensorReadingMap);
        if (config.controlMethod == AlifeControlMethod::EvolvedGauci)
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
        // Compute the centroid of puck positions.
        Vec2 sum(0.0, 0.0);
        for (const auto& puck : simWorldState->pucks)
            sum += puck.pos;
        Vec2 centroid(0.0, 0.0);
        double nPucks = simWorldState->pucks.size();
        if (simWorldState->pucks.size() > 0) {
            centroid.x = sum.x / nPucks;
            centroid.y = sum.y / nPucks;
        }

        // Now compute dispersion, the second moment of puck positions, which
        // corresponds to equation (2) from Gauci et al.  Effectively, it is
        // a scaled version of sum of squared distance of each puck from the centroid.
        double sumSquaredDistances = 0.0;
        for (const auto& puck : simWorldState->pucks)
            sumSquaredDistances += (puck.pos - centroid).lengthSquared();
        currentEvaluation = sumSquaredDistances / (4 * config.puckRadius * config.puckRadius);

        // Finally, update the cumulative sum of dispersion * time, which corresponds
        // to equation (3) from Gauci et al.
        cumulativeEvaluation += currentEvaluation * stepCount;
    }

    void evaluateDistanceToGoal()
    {
        // Compute the sum of squared distances of all pucks to the goal.
        double ssd = 0;
        for (const auto& puck : simWorldState->pucks)
            ssd += (puck.pos - simWorldState->goalPos).lengthSquared();            

        //if (simWorldState->nRobotRobotCollisions == 0 && simWorldState->nRobotBoundaryCollisions == 0)
            currentEvaluation = ssd;
        //else
        //    currentEvaluation = ssd + 1;

        // Finally, update the cumulative sum of distance * time.
        cumulativeEvaluation += currentEvaluation * stepCount;
    }
};