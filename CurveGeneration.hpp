#pragma once
#include <map>
//#include "tinysplinecxx.h"
#include "CommonTypes.hpp"
#include "WorldConfig.hpp"
#include "Voronoi.hpp"
#include "Track.hpp"
#include "Judgment.hpp"

using namespace CommonTypes;

namespace CurveGeneration {

//struct Curve {
//    std::vector<Vertex> vertices;
//};

using MapOfCurves = std::map<size_t, Track>;

using MapOfVectorOfCurves = std::map<size_t, std::vector<Track>>;

double artificialPointDistance = 1.5 * config.robotRadius;

std::vector<Track> curvesFromDilatedPolygons(std::vector<Voronoi::DilatedPolygon> polygons)
{
    std::vector<Track> curves;
    for (const Voronoi::DilatedPolygon &polygon : polygons)
    {
        Track curve;

        double lastX, lastY;
        bool lastValid = false;
        for (const auto &pt : polygon.vertices) {
            double x = pt.x;
            double y = pt.x;
            if (lastValid) {
                double dx = pt.x - lastX;
                double dy = pt.y - lastY;
                double theta = std::atan2(dy, dx);
                curve.poses.push_back({x, y, theta});
            }
            lastX = x;
            lastY = y;
            lastValid = true;
        }

        curves.push_back(curve);
    }

    return curves;
}

/*
std::vector<Track> splineCurvesFromSkeletons(std::vector<Voronoi::Skeleton> skeletons)
{
    std::vector<Track> curves;
    for (const Voronoi::Skeleton &skeleton : skeletons)
    {
        Track curve;

        std::vector<tinyspline::real> controlPoints = {
            skeleton.beginningPose.x, skeleton.beginningPose.y
        };

        // Augment the skeleton with an artificial point located a fixed distance
        // ahead of the robot.
        //double artificialX = skeleton.beginningPose.x + artificialPointDistance * cos(skeleton.beginningPose.theta);
        //double artificialY = skeleton.beginningPose.y + artificialPointDistance * sin(skeleton.beginningPose.theta);
        //controlPoints.push_back(artificialX);
        //controlPoints.push_back(artificialY);

        for (int i=0; i<skeleton.vertices.size(); i++)
        {
            auto a = skeleton.vertices[i];
            controlPoints.push_back(a.x);
            controlPoints.push_back(a.y);
        }

        // Use the points to create a cubic natural spline
        //tinyspline::BSpline spline = tinyspline::BSpline::interpolateCubicNatural(controlPoints, 2);
        tinyspline::BSpline spline = tinyspline::BSpline::interpolateCatmullRom(controlPoints, 2).toBeziers();

        // Generate curve vertices
        for (float u = 0; u <= 1; u += 0.01f) {
            auto position = spline.eval(u).resultVec2();
            auto derivative = spline.derive().eval(u).resultVec2();

            curve.poses.push_back({position.x(), position.y(), std::atan2(derivative.y(), derivative.x())});
        }

        curves.push_back(curve);
    }

    return curves;
}
*/

// Judges all curves in curvesMap and also returns the best curve for each robot.
Track judgeCurves(int robotIndex, std::vector<Track> &tracks, std::shared_ptr<WorldState> worldState)
{
    // Judge all tracks.
    for (auto& track : tracks) {
        // Judge the track using a copy of the world state.
        auto worldStateToJudge = std::make_shared<WorldState>(*worldState);
        Judgment::judgeTrack(track, robotIndex, worldStateToJudge);
    }

    // Now find the best track for this robot.
    double maxScore = -std::numeric_limits<double>::max();
    Track* bestTrack = nullptr;
    for (auto& track : tracks) {
        if (track.score > maxScore) {
            maxScore = track.score;
            bestTrack = &track;
        }
    }
    cout << "bestTrack->score: " << bestTrack->score << endl;

    return *bestTrack;
}

}; // namespace CurveGeneration