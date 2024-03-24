#pragma once
#include <map>
#include <vector>
#include <geos/geom/GeometryFactory.h>
#include <geos/triangulate/VoronoiDiagramBuilder.h>
#include <geos/geom/CoordinateArraySequence.h>
#include "CommonTypes.hpp"
#include "WorldState.hpp"
#include "WorldConfig.hpp"
#include "Angles.hpp"

using namespace geos::geom;
using namespace geos::triangulate;

using DilatedPolygon = CommonTypes::DilatedPolygon;
using Vec2 = CommonTypes::Vec2;

namespace GeosVoronoi {

using MapOfVectorOfDilatedPolygons = std::map<size_t, std::vector<DilatedPolygon>>;

using MapOfCentroids = std::map<size_t, Vec2>;

class GeosVoronoiBuilder {
private:
    Envelope envelope;
    MapOfVectorOfDilatedPolygons mapOfDilatedPolygons;
    MapOfCentroids mapOfCentroids;

    const double siteShiftRate = 0.01;

public:
    GeosVoronoiBuilder() : envelope(0, config.width, 0, config.height)
    {
    }

    void compute(std::shared_ptr<WorldState> worldState) {
        computeDilatedPolygons(worldState);
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
            if (!polygon) {
                throw std::runtime_error("Voronoi diagram contains non-polygonal geometry.");
            }   

            // Clip the polygon with the envelope
            auto clippedPolygon = polygon->intersection(factory->toGeometry(&envelope).get());
            if (clippedPolygon == nullptr || clippedPolygon->isEmpty()) {
                cout << "clippedPolygon is NULL or empty" << endl;
                continue;
            }
            if (!clippedPolygon->isValid()) {
                cout << "clippedPolygon is NOT valid" << endl;
                continue;
            }

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
            CommonTypes::Vec2 centroidVertex = {centroid->getX(), centroid->getY()};
            mapOfCentroids[robotIndex] = centroidVertex;

            // Determine the shortest distance from robotPoint to the boundary of the polygon
            auto boundary = clippedPolygon->getBoundary();
            double robotToPolyDist = robotPoint->distance(boundary.get());
            // std::cout << "robotToPolyDist: " << robotToPolyDist << std::endl;

            // Determine the minimum distance between any vertex of the clipped
            // polygon and the centroid of the polygon.  This will be used to
            // determine the maximum dilation.
            double maxDistance = 0;
            for (int j = 0; j < clippedPolygon->getNumPoints(); ++j) {
                auto coord = clippedPolygon->getCoordinates()->getAt(j);
                double distance = std::sqrt((coord.x - centroid->getX()) * (coord.x - centroid->getX()) + (coord.y - centroid->getY()) * (coord.y - centroid->getY()));
                if (distance > maxDistance) {
                    maxDistance = distance;
                }
            }

            // Choose the set of dilations we will produce.
            std::vector<double> dilations;
            for (double d = -config.robotRadius; d > -maxDistance; d -= config.dilationDelta) {
                dilations.push_back(d);
            }

            // DEBUG: Add the original polygon to the list of dilated polygons (with a dilation of 0)
            //dilations.push_back(0);
            //dilations.clear();
            //dilations.push_back(-100);

            //std::cout << "Dilations: ";
            //for (const auto& d : dilations) {
            //    std::cout << d << " ";
            //}
            //std::cout << std::endl;

            std::vector<DilatedPolygon> dilatedPolygons;
            for (auto d : dilations) {
                //cout << "Dilation: " << d << endl;
                DilatedPolygon dilatedPolygon;
                dilatedPolygon.dilation = d;

                //auto geosDilatedPolygon = clippedPolygon->buffer(d);
                //unique_ptr<geos::geom::Geometry> geosDilatedPolygon = clippedPolygon->buffer(d)->buffer(-50)->buffer(50);
                
                //cout << "BEFORE geosPolygonProcessing" << endl;
                int nPoints = clippedPolygon->getNumPoints();
                //cout << "clippedPolygon->getNumPoints(): " << nPoints << endl;
                auto geosDilatedPolygon = geosPolygonProcessing(factory, clippedPolygon, d, robotPoint, robot);
                if (geosDilatedPolygon == nullptr) {
                    continue;
                }
                nPoints = geosDilatedPolygon->getNumPoints();
                //cout << "geosDilatedPolygon->getNumPoints(): " << nPoints << endl;
                //cout << "AFTER geosPolygonProcessing" << endl;

                // Note that we ARE including the last point, which is the same as the first
                
                for (int j = 0; j < nPoints; ++j) {
//if (j > 20) break;
                    auto coord = geosDilatedPolygon->getCoordinates()->getAt(j);
                    dilatedPolygon.vertices.push_back(CommonTypes::Vec2(coord.x, coord.y));
                }

                if (dilatedPolygon.vertices.size() >= 3)
                    dilatedPolygons.push_back(dilatedPolygon);
            }

            // std::cout << "Dilated polygons: " << dilatedPolygons.size() << std::endl;
            
            if (!dilatedPolygons.empty()) {
                mapOfDilatedPolygons[robotIndex] = dilatedPolygons;
            }
        }
        // std::cout << "END OF COMPUTE DILATED POLYGONS " << std::endl;
    }

    unique_ptr<geos::geom::Geometry> geosPolygonProcessing(GeometryFactory::Ptr &factory, unique_ptr<geos::geom::Geometry> &clippedPolygon, double dilation, geos::geom::Point *robotPoint, Robot &robot) {
        //cout << "geosPolygonProcessing: A" << endl;
        //auto geosDilatedPolygon = clippedPolygon->buffer(dilation);
        //auto geosDilatedPolygon = clippedPolygon->buffer(dilation)->buffer(-50)->buffer(50);
        auto geosDilatedPolygon = clippedPolygon->buffer(dilation)->buffer(-5)->buffer(5);
        if (geosDilatedPolygon->getNumPoints() < 3)
            return nullptr;
        if (!geosDilatedPolygon->isValid()) {
            cout << "geosDilatedPolygon is NOT valid" << endl;
            return nullptr;
        }
        //cout << "geosDilatedPolygon->getNumPoints(): " << geosDilatedPolygon->getNumPoints() << endl;

        return geosDilatedPolygon;
    }
/*
    geos::geom::Point* findClosestPointOnPolygon(unique_ptr<geos::geom::Geometry> &polygon, geos::geom::Point *point) {
        //cout << "findClosestPointOnPolygon START" << endl;
        // Find the closest edge of the polygon to the given point.
        auto closestEdgeIndices = findClosestEdgeToPoint(polygon, point);
        CommonTypes::Vec2 first = {polygon->getCoordinates()->getAt(closestEdgeIndices.first).x, polygon->getCoordinates()->getAt(closestEdgeIndices.first).y};
        CommonTypes::Vec2 second = {polygon->getCoordinates()->getAt(closestEdgeIndices.second).x, polygon->getCoordinates()->getAt(closestEdgeIndices.second).y};
        //cout << "findClosestPointOnPolygon: AA" << endl;
        
        // We find the projection of the point's position onto the closest edge
        CommonTypes::Vec2 pointV = {point->getCoordinates()->getAt(0).x, point->getCoordinates()->getAt(0).y};
        CommonTypes::Vec2 a = pointV - first;
        CommonTypes::Vec2 b = {second.x - first.x, second.y - first.y};
        CommonTypes::Vec2 proj = first + b * (a.dot(b) / b.dot(b));
        //cout << "findClosestPointOnPolygon: BB" << endl;

        // Convert proj into a geos point.
        auto projPoint = GeometryFactory::getDefaultInstance()->createPoint({proj.x, proj.y});
        //cout << "findClosestPointOnPolygon - END" << endl;

        return projPoint;
    }

    std::pair<size_t, size_t> findClosestEdgeToPoint(unique_ptr<geos::geom::Geometry> &polygon, geos::geom::Point *point)
    {
        //cout << "findClosestEdgeToPoint START" << endl;
        std::pair<size_t, size_t> closestEdgeIndices = {-1, -1};
        double minDistance = std::numeric_limits<double>::max();
        //cout << "polygon->getNumPoints(): " << polygon->getNumPoints() << endl;
        for (int j = 0; j < polygon->getNumPoints(); ++j) {
            auto coord1 = polygon->getCoordinates()->getAt(j);
            auto coord2 = polygon->getCoordinates()->getAt((j + 1) % polygon->getNumPoints());
            auto edge = GeometryFactory::getDefaultInstance()->createLineString({coord1, coord2});
            double distance = edge->distance(point);
            if (distance < minDistance) {
                minDistance = distance;
                closestEdgeIndices = {j, (j + 1) % polygon->getNumPoints()};
            }
        }
        if (closestEdgeIndices.first == -1 || closestEdgeIndices.second == -1) {
            throw std::runtime_error("No closest edge found.");
        }
        //cout << "findClosestEdgeToPoint END" << endl;
        return closestEdgeIndices;
    }
    */
};

}; // namespace Voronoi