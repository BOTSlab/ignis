#pragma once
#include <geos/geom/GeometryFactory.h>
#include <geos/triangulate/VoronoiDiagramBuilder.h>
#include "CommonTypes.hpp"
#include "WorldState.hpp"
#include "WorldConfig.hpp"
#include "Angles.hpp"

using namespace geos::geom;
using namespace geos::triangulate;
using namespace CommonTypes;

namespace Voronoi {

struct DilatedPolygon {
    double dilation;
    std::vector<Vertex> vertices;
};

// A Skeleton represents the information extracted from a dilated polygon that
// will be used to form a Curve.
struct Skeleton {
    // The robot's pose is the beginning of every Skeleton.
    Pose begin;

    // The vertices are called 'middle' and 'end' because the beginning of
    // every track is the robot's pose.
    Vertex middle, end;
};

using MapOfVectorOfDilatedPolygons = std::map<size_t, std::vector<DilatedPolygon>>;

using MapOfVectorOfSkeletons = std::map<size_t, std::vector<Skeleton>>;

using MapOfCentroids = std::map<size_t, Vertex>;

class VoronoiBuilder {
private:
    Envelope envelope;
    MapOfVectorOfDilatedPolygons mapOfDilatedPolygons;
    MapOfVectorOfSkeletons mapOfSkeletons;
    MapOfCentroids mapOfCentroids;

    const double siteShiftRate = 0.01;

public:
    VoronoiBuilder() : envelope(0, config.width, 0, config.height)
    {
    }

    void compute(std::shared_ptr<WorldState> worldState) {
        computeDilatedPolygons(worldState);

        // From each dilated polygon we select a skeleton that will be used to form a track.
        computeSkeletons(worldState);
    }

    // Must be called after compute so that the centroids are up to date.
    void updateRobotSites(std::shared_ptr<WorldState> worldState) {
        // Loop over all vertices in mapOfCentroids
        for (const auto& pair : mapOfCentroids) {
            int robotIndex = pair.first;
            const auto& centroid = pair.second;
            auto &robot = worldState->robots[robotIndex];
            robot.siteX += siteShiftRate * (centroid.x - robot.siteX);
            robot.siteY += siteShiftRate * (centroid.y - robot.siteY);
        }
    }

    MapOfVectorOfDilatedPolygons getMapOfDilatedPolygons() {
        return mapOfDilatedPolygons;
    }

    MapOfVectorOfSkeletons getMapOfSkeletons() {
        return mapOfSkeletons;
    }

    MapOfCentroids getMapOfCentroids() {
        return mapOfCentroids;
    }
    
private:
    void computeDilatedPolygons(std::shared_ptr<WorldState> worldState)
    {
        mapOfDilatedPolygons.clear();
        mapOfCentroids.clear();

        GeometryFactory::Ptr factory = GeometryFactory::create();
        VoronoiDiagramBuilder builder;

        // Set the clip envelope to match the dimensions of the rectangular boundary.
        // Unfortunately, this is not enough to ensure that the polygons from the
        // Voronoi diagram are clipped, so we do that step in the next loop.
        builder.setClipEnvelope(&envelope);

        // Create a MultiPoint from the positions of the robots
        std::vector<Coordinate> coordinates;
        for (const auto& robot : worldState->robots) {
            coordinates.push_back(Coordinate(robot.siteX, robot.siteY));
        }
        auto multiPoint = factory->createMultiPoint(coordinates);

        // Set sites (coordinates) for Voronoi diagram
        auto coordSeq = multiPoint->getCoordinates();
        builder.setSites(*coordSeq);

        // Compute the Voronoi diagram
        auto diagram = builder.getDiagram(*factory);

        for (std::size_t i = 0; i < diagram->getNumGeometries(); ++i) {
            auto polygon = dynamic_cast<const Polygon*>(diagram->getGeometryN(i));
            if (polygon) {

                // Clip the polygon with the envelope
                auto clippedPolygon = polygon->intersection(factory->toGeometry(&envelope).get());

                // Determine the robot in worldState that lies within this polygon
                int robotIndex = -1;
                for (int j = 0; j < worldState->robots.size(); ++j) {
                    auto sitePoint = factory->createPoint(Coordinate(worldState->robots[j].siteX, worldState->robots[j].siteY));
                    if (clippedPolygon->contains(sitePoint)) {
                        robotIndex = j;
                    }
                }
                if (robotIndex == -1) {
                    throw std::runtime_error("No site found within Voronoi cell.");
                }

                auto robot = worldState->robots[robotIndex];
                auto robotPoint = factory->createPoint(Coordinate(worldState->robots[robotIndex].x, worldState->robots[robotIndex].y));

                // Determine the centroid of the clipped polygon
                auto centroid = clippedPolygon->getCentroid();
                Vertex centroidVertex = {centroid->getX(), centroid->getY()};
                mapOfCentroids.emplace(robotIndex, centroidVertex);

                // Determine the shortest distance from robotPoint to the boundary of the polygon
                auto boundary = clippedPolygon->getBoundary();
                double robotToPolyDist = robotPoint->distance(boundary.get());
                std::cout << "robotToPolyDist: " << robotToPolyDist << std::endl;

                // Choose the set of dilations we will produce.
                std::vector<double> dilations;
                double minDilation = -robotToPolyDist - config.halfDilationSteps * config.dilationDelta;
                double maxDilation = -robotToPolyDist + config.halfDilationSteps * config.dilationDelta;
                for (int i=-config.halfDilationSteps; i<=config.halfDilationSteps; i++) {
                    // We stop dilating if following the dilated polygon would intersect the original polygon
                    //if (d > -config.robotRadius)
                    //    break;
                    dilations.push_back(-robotToPolyDist + i * config.dilationDelta);
                }

                // DEBUG: Add the original polygon to the list of dilated polygons (with a dilation of 0)
                //dilations.push_back(0);

                std::cout << "Dilations: ";
                for (const auto& d : dilations) {
                    std::cout << d << " ";
                }
                std::cout << std::endl;

                std::vector<DilatedPolygon> dilatedPolygons;
                for (auto d : dilations) {
                    DilatedPolygon dilatedPolygon;
                    dilatedPolygon.dilation = d;

                    auto geosDilatedPolygon = clippedPolygon->buffer(d);
                    int nPoints = geosDilatedPolygon->getNumPoints();

                    // Note that we are not including the last point, which is the same as the first
                    for (int j = 0; j < nPoints - 1; ++j) {
                        auto coord = geosDilatedPolygon->getCoordinates()->getAt(j);
                        dilatedPolygon.vertices.push_back(Vertex(coord.x, coord.y));
                    }

                    if (dilatedPolygon.vertices.size() >= 3)
                        dilatedPolygons.push_back(dilatedPolygon);
                }

                std::cout << "Dilated polygons: " << dilatedPolygons.size() << std::endl;
                
                if (!dilatedPolygons.empty()) {
                    mapOfDilatedPolygons.emplace(robotIndex, dilatedPolygons);
                }
            }
        }
    }

    void computeSkeletons(std::shared_ptr<WorldState> worldState)
    {
        mapOfSkeletons.clear();

        // Go through all dilated polygons.  For each one, we will find the
        // two adjacent vertices that are closest to the robot's position.
        for (auto& pair : mapOfDilatedPolygons) {
            int robotIndex = pair.first;
            auto& dilatedPolygons = pair.second;
            auto& robot = worldState->robots[robotIndex];
            Pose robotPose = {robot.x, robot.y, robot.theta};

            std::vector<Skeleton> skeletons;
        
            for (std::size_t j = 0; j < dilatedPolygons.size(); ++j) {
                auto& dilatedPolygon = dilatedPolygons[j];
                auto& vertices = dilatedPolygon.vertices;

                // Determine the edge of the dilated polygon that is closest to the robot's position
                double minDistance = std::numeric_limits<double>::max();
                std::pair<Vertex, Vertex> closestEdge;
                std::pair<size_t, size_t> closestEdgeIndices;
                for (std::size_t k = 0; k < vertices.size(); ++k) {
                    auto& vertex1 = vertices[k];
                    size_t nextK = (k + 1) % vertices.size();
                    auto& vertex2 = vertices[nextK];

                    // Calculate the distance from the robot to the edge
                    double dx = vertex2.x - vertex1.x;
                    double dy = vertex2.y - vertex1.y;
                    double edgeLength = std::sqrt(dx * dx + dy * dy);
                    double t = ((robot.x - vertex1.x) * dx + (robot.y - vertex1.y) * dy) / (edgeLength * edgeLength);

                    // Clamp t between 0 and 1 to stay within the edge
                    t = std::max(0.0, std::min(1.0, t));

                    // Calculate the closest point on the edge to the robot
                    double closestX = vertex1.x + t * dx;
                    double closestY = vertex1.y + t * dy;

                    // Calculate the distance from the robot to the closest point
                    double distance = std::sqrt((closestX - robot.x) * (closestX - robot.x) + (closestY - robot.y) * (closestY - robot.y));

                    if (distance < minDistance) {
                        minDistance = distance;
                        closestEdge = {vertex1, vertex2};
                        closestEdgeIndices = {k, nextK};
                    }
                }
//skeletons.push_back({closestEdge.first, closestEdge.second});

                // Now that we have the closest edge, choose which of the two
                // vertices in this edge has the smallest relative angle from
                // the robot's orientation.
                double angle1 = std::atan2(closestEdge.first.y - robot.y, closestEdge.first.x - robot.x);
                double angle2 = std::atan2(closestEdge.second.y - robot.y, closestEdge.second.x - robot.x);

                double relativeAngle1 = Angles::getAngularDifference(angle1, robot.theta);
                double relativeAngle2 = Angles::getAngularDifference(angle2, robot.theta);
                //std::cout << "relativeAngle1: " << relativeAngle1 << " relativeAngle2: " << relativeAngle2 << std::endl;

                // The vertex with the smallest relative angle will be chosen as the middle vertex.
                // The end vertex will be the subsequent vertex from this one.
                //std::cout << "vertices.size(): " << vertices.size() << std::endl;
                if (relativeAngle1 < relativeAngle2) {
                    int indexDelta = closestEdgeIndices.first - closestEdgeIndices.second;
                    //std::cout << "indexDelta: " << indexDelta << std::endl;

                    size_t endIndex = (closestEdgeIndices.first + indexDelta) % vertices.size();
                    if (indexDelta == -1 && closestEdgeIndices.first == 0) {
                        endIndex = vertices.size() - 1;
                    }

                    //std::cout << "middle index: " << closestEdgeIndices.first << std::endl;
                    //std::cout << "end index: " << endIndex << std::endl;
                    skeletons.push_back({robotPose, closestEdge.first, vertices[endIndex]});
                } else {
                    int indexDelta = closestEdgeIndices.second - closestEdgeIndices.first;
                    //std::cout << "indexDelta: " << indexDelta << std::endl;
                    size_t endIndex = (closestEdgeIndices.second + indexDelta) % vertices.size();
                    if (indexDelta == -1 && closestEdgeIndices.second == 0) {
                        endIndex = vertices.size() - 1;
                    }
                    //std::cout << "middle index: " << closestEdgeIndices.second << std::endl;
                    //std::cout << "end index: " << endIndex << std::endl;
                    skeletons.push_back({robotPose, closestEdge.second, vertices[endIndex]});
                }
            }
            if (!skeletons.empty()) {
                mapOfSkeletons.emplace(robotIndex, skeletons);
            }
        }
    }
};

}; // namespace Voronoi