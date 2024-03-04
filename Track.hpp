#pragma once
#include <vector>
#include <map>
#include "WorldConfig.hpp"

struct Pose {
    double x;
    double y;
    double theta;
};

struct Track {
    // The baseScore is the score of the track before any judgment is applied.
    double baseScore;

    // The score is the score of the track after judgment is applied.
    double score;

    double turningRadius;

    bool best = false;

    std::vector<Pose> poses;

    void print() {
        std::cout << "baseScore: " << baseScore << std::endl;
        std::cout << "score: " << score << std::endl;
        std::cout << "turningRadius: " << turningRadius << std::endl;
    }
};

struct Plan : public Track {
    void incorporate(const Track& track) {
        /* THE SINGLE PUSH BACK / ERASE STRATEGY
        poses.push_back(track.poses[0]);

        if (poses.size() > config.planMaxLength)
            poses.erase(poses.begin());
        */

        // Add all poses from the track to the plan.
        for (auto& pose : track.poses) {
            poses.push_back(pose);
        }
    }

    void print() {
        for (auto& pose : poses) {
            std::cout << pose.x << " " << pose.y << " " << pose.theta << std::endl;
        }
    }
};

using VectorOfTrackVectors = std::vector< std::vector<Track> >;

using VectorOfPlans = std::vector< Plan >;