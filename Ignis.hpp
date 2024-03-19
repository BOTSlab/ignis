#pragma once
#include "Following.hpp"
#include "Sim.hpp"
#include "WorldConfig.hpp"
#include "worldInitializer.hpp"
#include "GeosVoronoi.hpp"
#include "CurveGeneration.hpp"

class Ignis {
public:
    WorldConfig config;
    GeosVoronoi::GeosVoronoiBuilder voronoiBuilder;

    std::shared_ptr<WorldState> simWorldState;
    GeosVoronoi::MapOfVectorOfDilatedPolygons robotIndexToDilatedPolygonsMap;
    CurveGeneration::MapOfVectorOfCurves robotIndexToCurvesMap;
    CurveGeneration::MapOfCurves robotIndexToBestCurveMap;

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

        for (int i = 0; i < config.numberOfRobots; ++i)
        {
            // First determine whether we should do any processing for this robot.
            // We will only consider skipping if the robot already has a best curve.
            bool skip = false;
            if (robotIndexToDilatedPolygonsMap.find(i) == robotIndexToDilatedPolygonsMap.end()) {
                // The robot has no dilated polygon to judge
                skip = true;
            } else {
                bool bestCurveExists = robotIndexToBestCurveMap.find(i) != robotIndexToBestCurveMap.end();
                if (bestCurveExists) {
                    auto& bestCurve = robotIndexToBestCurveMap.at(i);
                    if (!bestCurve.poses.empty()) {
                        /*
                        // If the robot already has a best curve and has not reached its
                        // end, then we can skip it.
                        skip = true;
                    } else {
                        */
                        // Skip if the robot is far from the end of its curve.
                        auto& lastPose = bestCurve.poses.back();
                        double dx = simWorldState->robots[i].x - lastPose.x;
                        double dy = simWorldState->robots[i].y - lastPose.y;
                        double distance = std::sqrt(dx * dx + dy * dy);
                        if (distance > config.robotRadius * 2)
                            skip = true;
                    }
                }
            }
            if (skip) {
                cout << "Skipping robot " << i << endl;
                continue;
            }
skip = false;
            cout << "Robot " << i << " is generating curves." << endl;
            robotIndexToCurvesMap[i] = CurveGeneration::curvesFromDilatedPolygons(robotIndexToDilatedPolygonsMap.at(i), simWorldState->robots[i]);
            if (robotIndexToCurvesMap[i].empty()) {
                cout << "Robot " << i << " has no curves to judge." << endl;
            } else {
                robotIndexToBestCurveMap[i] = CurveGeneration::judgeCurves(i, robotIndexToCurvesMap.at(i), simWorldState);
            }

            // paused = true;
        }
        // cout << "robotIndexToBestCurveMap.size(): " << robotIndexToBestCurveMap.size() << endl;
        if (robotIndexToBestCurveMap.size() == 0) {
            cout << "No best curves found." << endl;
            return;
        }

        Following::updateControlInputs(simWorldState, robotIndexToBestCurveMap);

        consumeBestCurves();

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

private:
    void consumeBestCurves()
    {
        for (auto& pair : robotIndexToBestCurveMap) {
            int robotIndex = pair.first;
            auto& bestCurve = pair.second;

            auto firstPose = bestCurve.poses.begin();
            double dx = simWorldState->robots[robotIndex].x - firstPose->x;
            double dy = simWorldState->robots[robotIndex].y - firstPose->y;
            double distance = std::sqrt(dx * dx + dy * dy);

            // If the closest point on the curve lies within the body of the robot, remove that point from the curve
            if (distance <= config.robotRadius)
                bestCurve.poses.erase(firstPose);
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