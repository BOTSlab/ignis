#pragma once
#include <map>
#include "tinysplinecxx.h"
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

MapOfVectorOfCurves curvesFromSkeletons(Voronoi::MapOfVectorOfSkeletons robotIndexToSkeletonsMap)
{
    MapOfVectorOfCurves curvesMap;

    for (const auto &pair : robotIndexToSkeletonsMap)
    {
        size_t robotIndex = pair.first;
        const auto &skeletons = pair.second;

        std::vector<Track> curves;
        for (const Voronoi::Skeleton &skeleton : skeletons)
        {
            Track curve;

            // The skeleton contains the start, middle, and end points, but we
            // will augment this with an artificial point located a fixed distance
            // ahead of the robot.
            double artificialX = skeleton.begin.x + artificialPointDistance * cos(skeleton.begin.theta);
            double artificialY = skeleton.begin.y + artificialPointDistance * sin(skeleton.begin.theta);

            std::vector<tinyspline::real> controlPoints = {
                skeleton.begin.x, skeleton.begin.y, // Start point
                artificialX, artificialY,
                skeleton.middle.x, skeleton.middle.y, // Control point
                skeleton.end.x, skeleton.end.y // End point
            };

            // Use the points to create a cubic natural spline
            //tinyspline::BSpline spline = tinyspline::BSpline::interpolateCubicNatural(controlPoints, 2);
            tinyspline::BSpline spline = tinyspline::BSpline::interpolateCatmullRom(controlPoints, 2);

            // Generate curve vertices
            for (float u = 0; u <= 1; u += 0.01f) {
                auto position = spline.eval(u).resultVec2();
                auto derivative = spline.derive().eval(u).resultVec2();

                curve.poses.push_back({position.x(), position.y(), std::atan2(derivative.y(), derivative.x())});
            }

            curves.push_back(curve);
        }
        curvesMap.emplace(robotIndex, curves);
    }

    return curvesMap;
}

// Judges all curves in curvesMap and also returns the best curve for each robot.
CurveGeneration::MapOfCurves judgeCurves(MapOfVectorOfCurves curvesMap, std::shared_ptr<WorldState> worldState)
{
    MapOfCurves bestCurves;

    for (auto& pair : curvesMap) {
        size_t robotIndex = pair.first;
        auto& tracks = pair.second;

        // Judge all tracks.
        for (auto& track : tracks) {
            // Judge the track using a copy of the world state.
            auto worldStateToJudge = std::make_shared<WorldState>(*worldState);
            Judgment::judgeTrack(track, robotIndex, worldStateToJudge);
        }

        // Now find the best track for this robot.
        double maxScore = -1;
        Track* bestTrack = nullptr;
        for (auto& track : tracks) {
            if (track.score > maxScore) {
                maxScore = track.score;
                bestTrack = &track;
            }
        }

        if (bestTrack != nullptr) {
            bestCurves.emplace(robotIndex, *bestTrack);
        }
    }

    return bestCurves;
}

}; // namespace CurveGeneration