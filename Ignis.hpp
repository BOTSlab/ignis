#pragma once
#include "Following.hpp"
#include "Sim.hpp"
#include "TrackGeneration.hpp"
#include "WorldConfig.hpp"
#include "worldInitializer.hpp"
#include "GeosVoronoi.hpp"
#include "CurveGeneration.hpp"

class Ignis {
public:
    WorldConfig config;
    //Voronoi::VoronoiBuilder voronoiBuilder;
    GeosVoronoi::GeosVoronoiBuilder voronoiBuilder;

    std::shared_ptr<WorldState> simWorldState;
    //std::shared_ptr<VectorOfTrackVectors> robotIndexToTracks;
    //std::shared_ptr<VectorOfPlans> robotIndexToPlans;
    GeosVoronoi::MapOfVectorOfDilatedPolygons robotIndexToDilatedPolygonsMap;
    //Voronoi::MapOfVectorOfSkeletons robotIndexToSkeletonsMap;
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
        //robotIndexToSkeletonsMap.clear();
        robotIndexToCurvesMap.clear();

        // BAD: The work done by the following functions is for all robots, but
        // we don't need to compute anything for robots that are not close to
        // the end of their current curves.
        voronoiBuilder.compute(simWorldState);
        voronoiBuilder.updateRobotSites(simWorldState);
        robotIndexToDilatedPolygonsMap = voronoiBuilder.getMapOfDilatedPolygons();
        //robotIndexToSkeletonsMap = voronoiBuilder.getMapOfSkeletons();

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
                        /*
                        auto& lastPose = bestCurve.poses.back();
                        double dx = simWorldState->robots[i].x - lastPose.x;
                        double dy = simWorldState->robots[i].y - lastPose.y;
                        double distance = std::sqrt(dx * dx + dy * dy);
                        if (distance > config.robotRadius * 2)
                            skip = true;
                        */
                    }
                }
            }
            if (skip) {
                cout << "Skipping robot " << i << endl;
                continue;
            }

            cout << "Robot " << i << " is generating curves." << endl;
            robotIndexToCurvesMap[i] = CurveGeneration::curvesFromDilatedPolygons(robotIndexToDilatedPolygonsMap.at(i));
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
    /*
    void arcTrackPlanManagement()
    {
        // GOOD IDEA?  This is a bit of a hack.  Prepare to store the tracks
        // for all robots, yet we may not have tracks for all robots.  Perhaps
        // switch to a map?
        robotIndexToTracks = std::make_shared<VectorOfTrackVectors>();
        for (int i = 0; i < config.numberOfRobots; ++i)
            robotIndexToTracks->push_back(std::vector<Track>());

        for (int i = 0; i < config.numberOfRobots; ++i)
        {
            // Determine whether the plan needs to be updated.
            bool update = false;

            // BAD: Making a copy of the plan just to judge it.  This is a waste
            // of memory and time.
            std::shared_ptr<Plan> planPtr = std::make_shared<Plan>((*robotIndexToPlans)[i]);

            auto worldStateToJudge = std::make_shared<WorldState>(*simWorldState);
            Judgment::judgeTrack(planPtr, i, worldStateToJudge);
            if (planPtr->score == -1)
            {
                (*robotIndexToPlans)[i].poses.clear();
                double x = simWorldState->robots[i].x;
                double y = simWorldState->robots[i].y;
                double theta = simWorldState->robots[i].theta;
                (*robotIndexToPlans)[i].poses.push_back({x, y, theta});
                update = true;
            }

            // Determine the distance to the last point in this robot's plan.  If the distance is less than a threshold, then update the plan.
            if (!update)
            {
                auto &plan = (*robotIndexToPlans)[i];
                auto &lastPose = plan.poses.back();
                double dx = simWorldState->robots[i].x - lastPose.x;
                double dy = simWorldState->robots[i].y - lastPose.y;
                double distance = std::sqrt(dx * dx + dy * dy);
                if (distance < config.robotRadius + 2 * config.sampleSpacing)
                {
                    update = true;
                }
            }

            if (update)
                updatePlan(i);
        }
    }
    */

    void reset()
    {
        simWorldState = worldInitializer();

        /*
        robotIndexToPlans = std::make_shared<VectorOfPlans>();
        for (int i = 0; i < config.numberOfRobots; ++i) {
            double x = simWorldState->robots[i].x;
            double y = simWorldState->robots[i].y;
            double theta = simWorldState->robots[i].theta;

            //robotIndexToPlans->push_back(Plan(x, y, theta));
            Plan plan;
            plan.poses.push_back({x, y, theta});
            robotIndexToPlans->push_back(plan);
        }
        */
        robotIndexToDilatedPolygonsMap.clear();
        //robotIndexToSkeletonsMap.clear();
        robotIndexToCurvesMap.clear();
        robotIndexToBestCurveMap.clear();
    }

    /*
    void updatePlan(int robotIndex) 
    {
        //std::cout << "Updating plan for robot " << robotIndex << std::endl;

        TrackGeneration::generateTracksForRobot(robotIndex, robotIndexToTracks, robotIndexToPlans, simWorldState);

        // Determine a pointer to the best track.
        auto& tracks = (*robotIndexToTracks)[robotIndex];

        // Loop through all tracks and determine a pointer to the one with the highest score.
        auto bestTrack = std::max_element(tracks.begin(), tracks.end(), [](const Track& a, const Track& b) {
            if (a.score == -1 && b.score == -1) {
                return a.turningRadius > b.turningRadius;
            }
            return a.score < b.score;
        });
        if (bestTrack == tracks.end()) {
            std::cout << "No best track found." << std::endl;
        }

        // Incorporate the best track into the plan.
        bestTrack->best = true;
        //bestTrack->print();
        (*robotIndexToPlans)[robotIndex].poses.clear();
        (*robotIndexToPlans)[robotIndex].incorporate(*bestTrack);

        //(*robotIndexToPlans)[robotIndex].print();
    }
    */
};