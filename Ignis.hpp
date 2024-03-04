#pragma once
#include "CurveFollowing.hpp"
#include "Sim.hpp"
#include "TrackGeneration.hpp"
#include "WorldConfig.hpp"
#include "worldInitializer.hpp"

class Ignis {
public:
    WorldConfig config;
    std::shared_ptr<WorldState> simWorldState;
    std::shared_ptr<VectorOfTrackVectors> robotIndexToTracks;
    std::shared_ptr<VectorOfPlans> robotIndexToPlans;
    int stepCount = 0;

    Ignis()
        : config()
    {
        reset();
    }

    void step()
    {
        Sim::update(simWorldState);

        // GOOD IDEA?  This is a bit of a hack.  Prepare to store the tracks
        // for all robots, yet we may not have tracks for all robots.  Perhaps
        // switch to a map?
        robotIndexToTracks = std::make_shared<VectorOfTrackVectors>();
        for (int i = 0; i < config.numberOfRobots; ++i)
            robotIndexToTracks->push_back(std::vector<Track>());

        for (int i = 0; i < config.numberOfRobots; ++i) {
            // Determine whether the plan needs to be updated.
            bool update = false;

            // BAD: Making a copy of the plan just to judge it.  This is a waste
            // of memory and time.
            std::shared_ptr<Plan> planPtr = std::make_shared<Plan>((*robotIndexToPlans)[i]);

            auto worldStateToJudge = std::make_shared<WorldState>(*simWorldState);
            Judgment::judgeTrack(planPtr, i, worldStateToJudge);
            if (planPtr->score == -1) {
                (*robotIndexToPlans)[i].poses.clear();
                double x = simWorldState->robots[i].x;
                double y = simWorldState->robots[i].y;
                double theta = simWorldState->robots[i].theta;
                (*robotIndexToPlans)[i].poses.push_back({x, y, theta});
                update = true;
            }

            // Determine the distance to the last point in this robot's plan.  If the distance is less than a threshold, then update the plan.
            if (!update) {
                auto& plan = (*robotIndexToPlans)[i];
                auto& lastPose = plan.poses.back();
                double dx = simWorldState->robots[i].x - lastPose.x;
                double dy = simWorldState->robots[i].y - lastPose.y;
                double distance = std::sqrt(dx * dx + dy * dy);
                if (distance < config.robotRadius + 2 * config.sampleSpacing) {
                    update = true;
                }
            }

            if (update)
                updatePlan(i);
        }

        CurveFollowing::updateControlInputs(simWorldState, robotIndexToPlans);

        stepCount++;
    }

    void reset()
    {
        simWorldState = worldInitializer();

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
    }

private:
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
};