#pragma once
#include "Track.hpp"
#include "WorldConfig.hpp"
#include <assert.h>
//#include "rapidcsv.h"

namespace TrackGeneration {

const double maxTrackLength = 500;
const double deltaAlpha = 0.05;
const double numberSamplesForStraightLine = 50;

std::shared_ptr<Track> arcTrack(double startX, double startY, double startTheta, double radius) {

    if (radius == 0) {
        // If the radius is 0, then we are generating a straight line.
        auto track = std::make_shared<Track>();
        for (double t = 0; t < maxTrackLength; t += maxTrackLength / numberSamplesForStraightLine) {
            double x = startX + t * cos(startTheta);
            double y = startY + t * sin(startTheta);
            track->points.push_back({x, y});
        }
        return track;
    }

    // If the radius is negative, then we know we are generating an arc on the right.
    bool left = radius > 0;

    // We'll now ensure the radius is positive.
    radius = fabs(radius);

    auto track = std::make_shared<Track>();
    track->points.push_back({startX, startY});

    // Sample along the arc until we reach the maximum arc length.
    double x, y, arcLength = 0;
    for (double alpha = 0; arcLength < maxTrackLength && alpha <= M_PI; alpha += deltaAlpha) {

        if (left) {
            x = startX + radius * (cos(startTheta + M_PI/2.0) + cos(startTheta - M_PI/2.0 + alpha));
            y = startY + radius * (sin(startTheta + M_PI/2.0) + sin(startTheta - M_PI/2.0 + alpha));
        } else {
            x = startX + radius * (cos(startTheta - M_PI/2.0) + cos(startTheta + M_PI/2.0 - alpha));
            y = startY + radius * (sin(startTheta - M_PI/2.0) + sin(startTheta + M_PI/2.0 - alpha));
        }
        track->points.push_back({x, y});

        arcLength += radius * deltaAlpha;
    }

    // Assign the track a base score based on the turning radius.
    track->score = - 1.0 / radius;

    return track;
}

/*
const double delta_t = 0.01;

std::shared_ptr<Track> quadBezierTrack(double startX, double startY, double startTheta, double goalX, double goalY) {
    std::cout << "\nNEW TRACK " << std::endl;

    auto track = std::make_shared<Track>();
    track->points.push_back({startX, startY});

    // Control point for the quadratic bezier curve
    double controlX = (startX + goalX) / 2 + (goalY - startY) * tan(startTheta) / 2;
    double controlY = (startY + goalY) / 2 - (goalX - startX) * tan(startTheta) / 2;

    // Sample along the bezier curve
    for (double t = 0; t <= 1; t += delta_t) {
        double x = (1 - t) * (1 - t) * startX + 2 * (1 - t) * t * controlX + t * t * goalX;
        double y = (1 - t) * (1 - t) * startY + 2 * (1 - t) * t * controlY + t * t * goalY;

        std::cout << "x: " << x << ", y: " << y << std::endl;
        
        track->points.push_back({x, y});
    }

    return track;
}
*/

/*
const double delta_t = 1.0;
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
*/

// Based on Smooth Controller 1 from 2018 notes for COMP 4766. 
/*
const double K_v = 10;
const double K_omega = 0.005;
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
*/

}; // namespace TrackGeneration