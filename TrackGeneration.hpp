#pragma once
#include "Track.hpp"
#include "WorldConfig.hpp"
#include "rapidcsv.h"

namespace TrackGeneration {

const double delta_t = 1.0;
const double K_v = 10;
const double K_omega = 0.005;

std::shared_ptr<Track> lookupTableTrack(double startX, double startY, double startTheta, double goalX, double goalY) {

    static rapidcsv::Document doc("lookup_table.csv");

    std::vector<double> xs = doc.GetColumn<double>("x");
    std::vector<double> ys = doc.GetColumn<double>("y");
    std::vector<double> thetas = doc.GetColumn<double>("yaw");

    auto track = std::make_shared<Track>();
    track->points.push_back({startX, startY});

    for (size_t i = 0; i < xs.size(); ++i) {
        double x = startX + xs[i] * cos(startTheta) - ys[i] * sin(startTheta);
        double y = startY + xs[i] * sin(startTheta) + ys[i] * cos(startTheta);
        track->points.push_back({x, y});
    }

    return track;
}

// Based on Smooth Controller 1 from 2018 notes for COMP 4766. 
std::shared_ptr<Track> smoothController1(double startX, double startY, double startTheta, double goalX, double goalY) {
    auto track = std::make_shared<Track>();
    track->points.push_back({startX, startY});

    double x = startX;
    double y = startY;
    double theta = startTheta;

    // Complete a fixed number of virtual movements towards the goal.  If we
    // reach the goal before the number of movements is complete, we stop.
    double distance = std::numeric_limits<double>::max();
    for (int i=0; i < config.maxTrackPoints && distance > 10; i++) {
        // Get the goal position in the robot's ref. frame.
        double goalRobRefX = goalX - x;
        double goalRobRefY = goalY - y;
        double goalRobRefXRot = goalRobRefX * cos(-theta) - goalRobRefY * sin(-theta);
        double goalRobRefYRot = goalRobRefX * sin(-theta) + goalRobRefY * cos(-theta);

        // Smooth controller 1 (from old 4766 notes) generates the following
        // forward and angular speeds
        double v = K_v; // * fabs(goalRobRefXRot);
        double w = K_omega * goalRobRefYRot;

        // Velocity in the global frame.
        double xDot = v * cos(theta);
        double yDot = v * sin(theta);

        x += xDot * delta_t;
        y += yDot * delta_t;
        theta += w * delta_t;

        track->points.push_back({x, y});

        distance = sqrt(pow(goalX - x, 2) + pow(goalY - y, 2));
        //printf("distance: %g\n", distance);
    }

    //printf("distance: %g, points: %d\n", distance, (int)track->points.size());
    return track;
}

}; // namespace TrackGeneration