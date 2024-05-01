#pragma once
#include <map>
#include "ignis/CurveTypes.hpp"
#include "common/Config.hpp"
#include "common/GeosVoronoi.hpp"
#include "ignis/Judgment.hpp"

using namespace CurveTypes;

namespace CurvesFromDilatedPolygons {

double polygonKeepProportion = 0.25;

double verticesPerUnitLength = 0.5;

int maxCurvePoints = 1000000;

// Resample the polygon to have verticesPerUnitLength vertices per unit length.
void resamplePolygon(DilatedPolygon &polygon)
{
    // Calculate the perimeter of the polygon
    double perimeter = 0.0;
    std::vector<double> segmentLengths;
    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
        size_t next_i = (i + 1) % polygon.vertices.size();
        double dx = polygon.vertices[next_i].x - polygon.vertices[i].x;
        double dy = polygon.vertices[next_i].y - polygon.vertices[i].y;
        double segmentLength = std::sqrt(dx * dx + dy * dy);
        perimeter += segmentLength;
        segmentLengths.push_back(segmentLength);
    }

    // Calculate the number of vertices to generate
    size_t numVertices = static_cast<size_t>(std::round(perimeter * verticesPerUnitLength));

    // Generate the new vertices
    std::vector<Vec2> newVertices;
    double step = perimeter / numVertices;
    double distance = 0.0;
    for (size_t i = 0; i < numVertices; ++i) {
        while (distance > segmentLengths[0]) {
            distance -= segmentLengths[0];
            segmentLengths.erase(segmentLengths.begin());
            polygon.vertices.erase(polygon.vertices.begin());
        }
        double alpha = distance / segmentLengths[0];
        double x = (1 - alpha) * polygon.vertices[0].x + alpha * polygon.vertices[1].x;
        double y = (1 - alpha) * polygon.vertices[0].y + alpha * polygon.vertices[1].y;
        newVertices.push_back({x, y});
        distance += step;
    }

    // Replace the old vertices with the new ones
    polygon.vertices = std::move(newVertices);
}

// Reorder the vertices of the polygon so that the first Vec2 is the one closest 
// to the robot's nose.
void reorderVertices(DilatedPolygon &polygon, Robot robot)
{
    double noseX = robot.pos.x + config.robotRadius * cos(robot.theta);
    double noseY = robot.pos.y + config.robotRadius * sin(robot.theta);

    // Find the Vec2 closest to the robot
    double minDistance = std::numeric_limits<double>::max();
    size_t closestIndex = 0;
    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
        double dx = polygon.vertices[i].x - noseX;
        double dy = polygon.vertices[i].y - noseY;
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i;
        }
    }

    // Reorder the vertices so that the closest one is first
    std::rotate(polygon.vertices.begin(), polygon.vertices.begin() + closestIndex, polygon.vertices.end());
}

// Check if the robot is oriented CW or CCW with respect to the polygon.  If the
// robot's orientation is opposite to the polygon's, then reverse the order of the
// vertices.
void reverseVec2OrderIfNecessary(DilatedPolygon &polygon, Robot robot)
{
    double noseX = robot.pos.x + config.robotRadius * cos(robot.theta);
    double noseY = robot.pos.y + config.robotRadius * sin(robot.theta);

    // Find the closest edge to the robot
    double minDistance = std::numeric_limits<double>::max();
    size_t closestIndex = 0;
    for (size_t i = 0; i < polygon.vertices.size(); ++i) {
        size_t next_i = (i + 1) % polygon.vertices.size();
        double dx = polygon.vertices[next_i].x - polygon.vertices[i].x;
        double dy = polygon.vertices[next_i].y - polygon.vertices[i].y;
        double edgeX = polygon.vertices[i].x + dx / 2;
        double edgeY = polygon.vertices[i].y + dy / 2;
        double dx2 = edgeX - noseX;
        double dy2 = edgeY - noseY;
        double distance = std::sqrt(dx2 * dx2 + dy2 * dy2);
        if (distance < minDistance) {
            minDistance = distance;
            closestIndex = i;
        }
    }

    // Find the angle between the robot's orientation and the closest edge
    size_t next_i = (closestIndex + 1) % polygon.vertices.size();
    double dx = polygon.vertices[next_i].x - polygon.vertices[closestIndex].x;
    double dy = polygon.vertices[next_i].y - polygon.vertices[closestIndex].y;
    double edgeTheta = std::atan2(dy, dx);
    double angle = Angles::getAngularDifference(robot.theta, edgeTheta);

    // If the absolute difference is greater than 90 degrees, then reverse the order of the vertices
    if (angle > M_PI / 2 || angle < -M_PI / 2) {
        std::reverse(polygon.vertices.begin(), polygon.vertices.end());
    }
}

// Define a normal distribution
double normalDistribution(double x, double variance)
{
    double mean = 0.0; // assuming mean as 0.0
//    return (1.0 / std::sqrt(2.0 * M_PI * variance)) * std::exp(-0.5 * (x - mean) * (x - mean) / variance);
return std::exp(-0.5 * (x - mean) * (x - mean) / variance);
}

// Locally distort the polygon so that it bulges inward's or outwards from the
// polygon's centorid to pass through the robot's nose.  The degree of bulging
// is determined by the distance to the nose.
void bulgePolygonToRobot(DilatedPolygon &polygon, Robot robot)
{
    double noseX = robot.pos.x + config.robotRadius * cos(robot.theta);
    double noseY = robot.pos.y + config.robotRadius * sin(robot.theta);

    CommonTypes::Vec2 nosePt = CommonTypes::Vec2(noseX, noseY);

    // Calculate the centroid of the polygon
    CommonTypes::Vec2 centroid{0.0, 0.0};
    for (const auto &pt : polygon.vertices) {
        centroid.x += pt.x;
        centroid.y += pt.y;
    }
    centroid.x /= polygon.vertices.size();
    centroid.y /= polygon.vertices.size();

    int i = 0;
    for (auto &pt : polygon.vertices) {
        double dx = pt.x - centroid.x;
        double dy = pt.y - centroid.y;
        double centroidToPointAngle = std::atan2(dy, dx);
        double centroidToPointDistance = std::sqrt(dx * dx + dy * dy);

        dx = noseX - centroid.x;
        dy = noseY - centroid.y;
        double centroidToRobotAngle = std::atan2(dy, dx);
        double centroidToRobotDistance = std::sqrt(dx * dx + dy * dy);
        double deltaDistance = centroidToRobotDistance - centroidToPointDistance;

        double angleDiff = Angles::getAngularDifference(centroidToPointAngle, centroidToRobotAngle);

        /* SORTA KINDA WORKS
        if (angleDiff < M_PI/8) {
            double newDistance = centroidToPointDistance + deltaDistance * normalDistribution(angleDiff, 0.1);
            pt.x = centroidX + newDistance * std::cos(centroidToPointAngle);
            pt.y = centroidY + newDistance * std::sin(centroidToPointAngle);
        }
        */

        /* GETTING CLOSER!
        CommonTypes::Vec2 robotPt = CommonTypes::Vec2(robot.x, robot.y);
        double factor = normalDistribution(angleDiff, 1);
        pt = pt * (1 - factor) + robotPt * factor;
        */

        /* SIMILAR TO THE ABOVE, BUT MORE SUCCINCT
        double factor = normalDistribution(angleDiff, 0.1);
        pt = centroid + (pt - centroid) * (1 - factor) + (robotPt - centroid) * factor;
        */

        /* This is the best so far!!
        double factor = normalDistribution(angleDiff, 0.1);
        pt.x = centroid.x + (centroidToPointDistance * (1 - factor) + centroidToRobotDistance * factor) * std::cos(centroidToPointAngle);
        pt.y = centroid.y + (centroidToPointDistance * (1 - factor) + centroidToRobotDistance * factor) * std::sin(centroidToPointAngle);
        */

        /* BETTER
        pt += (robotPt - pt) * (normalDistribution(angleDiff, 0.1));
        */

        // Smoothest and simplest!
        pt += (nosePt - pt) * normalDistribution(i, 5000);

        ++i;
    }
}

void processPolygon(DilatedPolygon &polygon, Robot robot)
{
    resamplePolygon(polygon);
    reorderVertices(polygon, robot);
    reverseVec2OrderIfNecessary(polygon, robot);

    if (config.curveBlendMethod == CurveBlendMethod::Bulge) {
        // Remove the last (1 - polygonKeepProportion) proportion of the vertices.
        size_t newSize = static_cast<size_t>(polygon.vertices.size() * polygonKeepProportion);
        polygon.vertices.resize(newSize);

        bulgePolygonToRobot(polygon, robot);
    } else {
        std::cerr << "processPolygon - unknown curve blend method" << std::endl;
    }
}

std::vector<Curve> curvesFromDilatedPolygons(std::vector<DilatedPolygon> polygons, Robot robot)
{
    std::vector<Curve> curves;
    for (DilatedPolygon &polygon : polygons)
    {
        Curve curve;

        processPolygon(polygon, robot);

        double lastX, lastY;
        bool lastValid = false;
        for (const auto &pt : polygon.vertices) {
            double x = pt.x;
            double y = pt.y;
            if (lastValid) {
                double dx = pt.x - lastX;
                double dy = pt.y - lastY;
                double theta = std::atan2(dy, dx);
                curve.points.push_back({x, y, theta});
            }
            if (curve.points.size() > maxCurvePoints) {
                std::cerr << "curvesFromDilatedPolygons - curve has too many points:" << curve.points.size() << std::endl;
                break;
            }
            lastX = x;
            lastY = y;
            lastValid = true;
        }

        curves.push_back(curve);
    }

    return curves;
}

}; // namespace CurvesFromDilatedPolygons