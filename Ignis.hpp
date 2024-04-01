#pragma once
#include "Sim.hpp"
#include "WorldConfig.hpp"
#include "worldInitializer.hpp"
#include "GeosVoronoi.hpp"
#include "CurvesFromDilatedPolygons.hpp"
#include "CurvesFromArcs.hpp"
#include "Following.hpp"

class Ignis {
public:
    WorldConfig config;
    GeosVoronoi::GeosVoronoiBuilder voronoiBuilder;

    std::shared_ptr<WorldState> simWorldState;
    GeosVoronoi::MapOfVectorOfDilatedPolygons robotIndexToDilatedPolygonsMap;
    MapOfVectorOfCurves robotIndexToCurvesMap;
    MapOfCurves robotIndexToBestCurveMap;

    // Simulation state variables.
    int stepCount = 0;
    bool paused = false;

    // These are not state variables, but rather flags that are set externally.
    bool doReset = false;
    bool doPause = false;
    bool doUnpause = false;
    bool doStepOnce = false;

    Ignis()
        : voronoiBuilder(), config()
    {
        reset();

        // Perform a number of steps to resolve any initial collisions.
        for (int i=0; i<config.coldStartSteps; i++)
            Sim::update(simWorldState);
    }

    void step()
    {
        if (doReset) {
            doReset = false;
            reset();
        }
        if (doPause) {
            doPause = false;
            paused = true;
        }
        if (doUnpause) {
            doUnpause = false;
            paused = false;
        }
        if (doStepOnce) {
            // We set doStepOnce to false and set paused to true at the end of this function.
            paused = false;
        }

        if (paused)
            return;

        Sim::update(simWorldState);

        robotIndexToDilatedPolygonsMap.clear();
        robotIndexToCurvesMap.clear();

        // BAD: The work done by the following functions is for all robots, but
        // we don't need to compute anything for robots that are not close to
        // the end of their current curves.
        voronoiBuilder.compute(simWorldState);
        voronoiBuilder.updateRobotSites(simWorldState);
        robotIndexToDilatedPolygonsMap = voronoiBuilder.getMapOfDilatedPolygons();

        updateBestCurves();

        Following::updateControlInputs(simWorldState, robotIndexToBestCurveMap);

        stepCount++;

        if (doStepOnce) {
            doStepOnce = false;
            paused = true;
        }
    }

    void prepareToReset()
    {
        doReset = true;
    }

    void prepareToPause()
    {
        doPause = true;
    }

    void prepareToUnpause()
    {
        doUnpause = true;
    }

    void prepareToStepOnce()
    {
        doStepOnce = true;
    }

    bool isPaused() const
    {
        return paused;
    }

    int getStepCount() const
    {
        return stepCount;
    }
    
private:
    void updateBestCurves()
    {
        for (int i = 0; i < config.numberOfRobots; ++i)
        {
            // First determine whether we should do any processing for this robot.
            // We will only consider skipping if the robot already has a best curve.
            bool bestCurveExists = robotIndexToBestCurveMap.find(i) != robotIndexToBestCurveMap.end();
            if (bestCurveExists && !robotIndexToBestCurveMap.at(i).isFinishedFollowing()) {
                // cout << "Skipping robot " << i << endl;
                continue;
            }

            cout << "Robot " << i << " is generating curves." << endl;

            if (config.overallMethod == OverallMethod::DilatedPolygonCurves) {
                robotIndexToCurvesMap[i] = CurvesFromDilatedPolygons::curvesFromDilatedPolygons(robotIndexToDilatedPolygonsMap.at(i), simWorldState->robots[i]);
            } else if (config.overallMethod == OverallMethod::ArcCurves) {
                robotIndexToCurvesMap[i] = CurvesFromArcs::arcCurvesForRobot(simWorldState->robots[i]);
            } else {
                cout << "Unknown overall method." << endl;
                return;
            }

            if (robotIndexToCurvesMap[i].empty()) {
                cout << "Robot " << i << " has no curves to judge." << endl;
            } else {
                robotIndexToBestCurveMap[i] = Judgment::judgeCurves(i, robotIndexToCurvesMap.at(i), simWorldState);
            }

            paused = true;

            // Reset the index to seek for the best curve.  This readies the robot
            // to follow this curve from its beginning to its end.  This is necessary
            // because judgement of this curve (i.e. simulation of its performance)
            // involved its own manipulation of the index to seek.
            robotIndexToBestCurveMap[i].setIndexToSeek(0);
        }
        // cout << "robotIndexToBestCurveMap.size(): " << robotIndexToBestCurveMap.size() << endl;
        if (robotIndexToBestCurveMap.size() == 0) {
            cout << "No best curves found." << endl;
            return;
        }
    }

    void reset()
    {
        simWorldState = worldInitializer();

        robotIndexToDilatedPolygonsMap.clear();
        robotIndexToCurvesMap.clear();
        robotIndexToBestCurveMap.clear();
    }
};