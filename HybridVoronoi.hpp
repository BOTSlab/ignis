/**
 * This is called HybridVoronoi because it uses a combination of GEOS and CGAL
 * to compute the Voronoi diagram and then dilate the cells to create the
 * dilated polygons.  The dilated polygons are then used to generate the curves
 * that the robots will follow.
*/

#pragma once
#include <map>

// GEOS includes
#include <geos/geom/GeometryFactory.h>
#include <geos/triangulate/VoronoiDiagramBuilder.h>
#include <geos/geom/CoordinateSequenceFactory.h>

// CGAL includes
#include <CGAL/Polygon_2.h>
#include <CGAL/Polygon_with_holes_2.h>
#include <CGAL/Boolean_set_operations_2.h>
#include <CGAL/centroid.h>
#include <CGAL/squared_distance_2.h>
#include <CGAL/create_offset_polygons_2.h>

#include "CommonTypes.hpp"
#include "WorldState.hpp"
#include "WorldConfig.hpp"
#include "Angles.hpp"

// Using fully qualified names for GEOS types, but CGAL's types are insane, so
// here some CGAL typedefs.
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef K::Point_2 CgalPoint;
typedef CGAL::Polygon_2<K> CgalPolygon;
typedef CGAL::Polygon_with_holes_2<K> CgalPolygonWithHoles;

namespace HybridVoronoi {

struct DilatedPolygon {
    std::vector<CommonTypes::Vertex> vertices;
};

using MapOfVectorOfDilatedPolygons = std::map<size_t, std::vector<DilatedPolygon>>;

using MapOfCentroids = std::map<size_t, CommonTypes::Vertex>;

class HybridVoronoiBuilder {
private:
    geos::geom::Envelope envelope;
    MapOfVectorOfDilatedPolygons mapOfDilatedPolygons;
    MapOfCentroids mapOfCentroids;

    const double siteShiftRate = 0.01;

public:
    HybridVoronoiBuilder() : envelope(0, config.width, 0, config.height)
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

        geos::geom::GeometryFactory::Ptr factory = geos::geom::GeometryFactory::create();
        geos::triangulate::VoronoiDiagramBuilder builder;

        // Set the clip envelope to match the dimensions of the rectangular boundary.
        // Unfortunately, this is not enough to ensure that the polygons from the
        // Voronoi diagram are clipped, so we do that step in the next loop.
        builder.setClipEnvelope(&envelope);

        // Create a MultiPoint from the positions of the robots
        std::vector<geos::geom::Coordinate> coordinates;
        for (const auto& robot : worldState->robots) {
            coordinates.push_back(geos::geom::Coordinate(robot.siteX, robot.siteY));
        }
        auto multiPoint = factory->createMultiPoint(coordinates);

        // Set sites (coordinates) for Voronoi diagram
        auto coordSeq = multiPoint->getCoordinates();
        builder.setSites(*coordSeq);

        // Compute the Voronoi diagram
        auto diagram = builder.getDiagram(*factory);

        for (std::size_t i = 0; i < diagram->getNumGeometries(); ++i) {
            auto geosPolygon = dynamic_cast<const geos::geom::Polygon*>(diagram->getGeometryN(i));
            if (!geosPolygon)
                throw std::runtime_error("Voronoi diagram contains a non-polygon geometry.");

            // Clip the polygon with the envelope
            auto clippedGeosPolygon = geosPolygon->intersection(factory->toGeometry(&envelope).get());

            // Convert the clipped polygon to a CGAL polygon
            CgalPolygon cgalPolygon;
            for (int j = 0; j < clippedGeosPolygon->getNumPoints(); ++j) {
                auto coord = clippedGeosPolygon->getCoordinates()->getAt(j);
                cgalPolygon.push_back(CgalPoint(coord.x, coord.y));
            }

            processVoronoiCell(cgalPolygon, worldState);
        }
    }

    void processVoronoiCell(CgalPolygon cell, std::shared_ptr<WorldState> worldState) {
        // Determine the index of the robot whose site lies within this cell.
        int robotIndex = -1;
        for (int i = 0; i < worldState->robots.size(); ++i) {
            // Check if the robot's position is within the polygon
            if (CGAL::bounded_side_2(cell.vertices_begin(), cell.vertices_end(), CgalPoint(worldState->robots[i].siteX, worldState->robots[i].siteY), K()) == CGAL::ON_BOUNDED_SIDE) {
                robotIndex = i;
                break;
            }
        }
        if (robotIndex == -1) {
            throw std::runtime_error("No site found within Voronoi cell.");
        }
cout << "D" << endl;

        // Compute the centroid of the cell
        auto centroid = CGAL::centroid(cell.vertices_begin(), cell.vertices_end());
        mapOfCentroids.emplace(robotIndex, CommonTypes::Vertex(centroid.x(), centroid.y()));

        // Create all of the inner offset polygons from this cell.
        auto offset_polygons = CGAL::create_exterior_skeleton_and_offset_polygons_2(config.dilationDelta, cell);
        cout << "number of offset polygons: " << offset_polygons.size() << endl;
cout << "E" << endl;

        // Loop through the offset polygons and create the dilated polygons
        std::vector<DilatedPolygon> dilatedPolygons;

        // For some reason, offset_polygons always seem to contain two polygons,
        // but the first one is strange.  The second one is the one we want.
        auto poly = offset_polygons[1];

        DilatedPolygon dilatedPolygon;
        for (auto v = poly->vertices_begin(); v != poly->vertices_end(); ++v)
            dilatedPolygon.vertices.push_back(CommonTypes::Vertex(v->x(), v->y()));
        dilatedPolygons.push_back(dilatedPolygon);

        if (!dilatedPolygons.empty())
            mapOfDilatedPolygons.emplace(robotIndex, dilatedPolygons);
cout << "G" << endl;
    }

/*
    unique_ptr<geos::geom::Geometry> geosPolygonProcessing(GeometryFactory::Ptr &factory, unique_ptr<geos::geom::Geometry> &clippedPolygon, double dilation, geos::geom::Point *robotPoint, Robot &robot) {
        auto geosDilatedPolygon = clippedPolygon->buffer(dilation);
        //auto geosDilatedPolygon = clippedPolygon->buffer(dilation)->buffer(-50)->buffer(50);

        // Define a point ahead of the robot.
        double aheadDistance = 100;
        double aheadX = robot.x + aheadDistance * std::cos(robot.theta);
        double aheadY = robot.y + aheadDistance * std::sin(robot.theta);
        auto aheadPoint = GeometryFactory::getDefaultInstance()->createPoint({aheadX, aheadY});

        // p1 is the closest point on the polygon to the robot's position.
        geos::geom::Point *p1 = findClosestPointOnPolygon(geosDilatedPolygon, robotPoint);

        // p2 is the closest point on the polygon to the point ahead of the robot.
        geos::geom::Point *p2 = findClosestPointOnPolygon(geosDilatedPolygon, aheadPoint);

        // Determine if the robot is inside the polygon.
        bool inside = geosDilatedPolygon->contains(robotPoint);

        if (inside) {
        } else {
            // Create a CoordinateArraySequence using the following points: robotPoint, aheadPoint, p1, p2, and robotPoint.
            std::vector<Coordinate> coordinates;
            coordinates.push_back(*(robotPoint->getCoordinate()));
            coordinates.push_back(*(aheadPoint->getCoordinate()));
            coordinates.push_back(*(p2->getCoordinate()));
            coordinates.push_back(*(p1->getCoordinate()));
            coordinates.push_back(*(robotPoint->getCoordinate()));
            cout << "A" << endl;

            // Create a CoordinateArraySequence from the vector of coordinates
            auto coordinateSequenceFactory = factory->getCoordinateSequenceFactory();
            auto coordSeq = coordinateSequenceFactory->create(coordinates.size(), 2);
            for (int i = 0; i < coordinates.size(); ++i) {
                coordSeq->setAt(coordinates[i], i);
            }
            cout << "B" << endl;

            // Create a LinearRing from the CoordinateArraySequence
            auto linearRing = factory->createLinearRing(coordSeq.get());

            cout << "C" << endl;
            // Create a Polygon from the LinearRing
            std::unique_ptr<Polygon> polygonPtr(factory->createPolygon(linearRing, nullptr));

            cout << "D" << endl;
            // Compute the union of geosDilatedPolygon and polygonPtr
            auto unionPolygon = geosDilatedPolygon->Union(polygonPtr.get());
            cout << "E" << endl;

            return unionPolygon;
        }
        
        return geosDilatedPolygon;
    }

    geos::geom::Point* findClosestPointOnPolygon(unique_ptr<geos::geom::Geometry> &polygon, geos::geom::Point *point) {
        // Find the closest edge of the polygon to the given point.
        auto closestEdgeIndices = findClosestEdgeToPoint(polygon, point);
        CommonTypes::Vertex first = {polygon->getCoordinates()->getAt(closestEdgeIndices.first).x, polygon->getCoordinates()->getAt(closestEdgeIndices.first).y};
        CommonTypes::Vertex second = {polygon->getCoordinates()->getAt(closestEdgeIndices.second).x, polygon->getCoordinates()->getAt(closestEdgeIndices.second).y};
        
        // We find the projection of the point's position onto the closest edge
        CommonTypes::Vertex pointV = {point->getCoordinates()->getAt(0).x, point->getCoordinates()->getAt(0).y};
        CommonTypes::Vertex a = pointV - first;
        CommonTypes::Vertex b = {second.x - first.x, second.y - first.y};
        CommonTypes::Vertex proj = first + b * (a.dot(b) / b.dot(b));

        // Convert proj into a geos point.
        auto projPoint = GeometryFactory::getDefaultInstance()->createPoint({proj.x, proj.y});

        return projPoint;
    }

    std::pair<size_t, size_t> findClosestEdgeToPoint(unique_ptr<geos::geom::Geometry> &polygon, geos::geom::Point *point)
    {
        std::pair<size_t, size_t> closestEdgeIndices;
        double minDistance = std::numeric_limits<double>::max();
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
        return closestEdgeIndices;
    }
*/
};

}; // namespace HybridVoronoi