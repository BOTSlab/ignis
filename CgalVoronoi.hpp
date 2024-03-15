#pragma once
#include <map>
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_2.h>
#include <CGAL/Voronoi_diagram_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_traits_2.h>
#include <CGAL/Delaunay_triangulation_adaptation_policies_2.h>
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

// Define the types for the Voronoi diagram
typedef CGAL::Exact_predicates_inexact_constructions_kernel K;
typedef CGAL::Delaunay_triangulation_2<K> Delaunay;

typedef CGAL::Exact_predicates_inexact_constructions_kernel                  K;
typedef CGAL::Delaunay_triangulation_2<K>                                    DT;
typedef CGAL::Delaunay_triangulation_adaptation_traits_2<DT>                 AT;
typedef CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<DT> AP;
typedef CGAL::Voronoi_diagram_2<DT,AT,AP> VoronoiDiagram;
//typedef CGAL::Voronoi_diagram_2<Delaunay, CGAL::Delaunay_triangulation_adaptation_traits_2<Delaunay>,
//                                 CGAL::Delaunay_triangulation_caching_degeneracy_removal_policy_2<Delaunay>> Voronoi;
typedef K::Point_2 Point;
typedef CGAL::Polygon_2<K> Polygon;
typedef CGAL::Polygon_with_holes_2<K> Polygon_with_holes;

namespace CgalVoronoi {

struct DilatedPolygon {
    std::vector<CommonTypes::Vertex> vertices;
};

using MapOfVectorOfDilatedPolygons = std::map<size_t, std::vector<DilatedPolygon>>;

using MapOfCentroids = std::map<size_t, CommonTypes::Vertex>;

class CgalVoronoiBuilder {
private:
    //Envelope envelope;
    MapOfVectorOfDilatedPolygons mapOfDilatedPolygons;
    MapOfCentroids mapOfCentroids;

    const double siteShiftRate = 0.01;

public:
    CgalVoronoiBuilder() //: envelope(0, config.width, 0, config.height)
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

        // Create a vector of points for the Voronoi diagram
        std::vector<Point> points;
        for (const auto& robot : worldState->robots) {
            points.push_back(Point(robot.siteX, robot.siteY));
        }

        // Construct the Delaunay triangulation
        Delaunay dt(points.begin(), points.end());

        // Construct the Voronoi diagram
        VoronoiDiagram voronoi(dt);
        
        cout << "A" << endl;

        // Iterate over the Voronoi cells
        for (auto fit = voronoi.unbounded_faces_begin(); fit != voronoi.unbounded_faces_end(); ++fit) {
            cout << "B" << endl;

            

            // Convert the Voronoi face to a Polygon

            Polygon polygon;
            cout << "1" << endl;
            auto start = fit->ccb();
            cout << "ccb->is_valid(): " << start->is_valid() << endl;
            auto current = start;
            cout << "2" << endl;
            do {
            cout << "3" << endl;
                const Point& p = current->source()->point();
            cout << "4" << endl;
                polygon.push_back(p);
            cout << "5" << endl;
                ++current;
            cout << "6" << endl;
            } while (current != start);
            cout << "7" << endl;
            
            
            cout << "C" << endl;
            processVoronoiCell(polygon, worldState);
        }
            cout << "D" << endl;
    }

    void processVoronoiCell(Polygon cell, std::shared_ptr<WorldState> worldState) {
        // Clip this polygon to the boundary of the world
        Polygon boundary;
        boundary.push_back(Point(0, 0));
        boundary.push_back(Point(config.width, 0));
        boundary.push_back(Point(config.width, config.height));
        std::list<Polygon_with_holes> intersection;
        CGAL::intersection(cell, boundary, std::back_inserter(intersection));
        if (!intersection.empty()) {
            // Overwrite the cell with the clipped polygon
            cell = intersection.front().outer_boundary();
        }

        // Determine the index of the robot whose site lies within this cell.
        int robotIndex = -1;
        for (int i = 0; i < worldState->robots.size(); ++i) {
            // Check if the robot's position is within the polygon
            if (CGAL::bounded_side_2(cell.vertices_begin(), cell.vertices_end(), Point(worldState->robots[i].siteX, worldState->robots[i].siteY), K()) == CGAL::ON_BOUNDED_SIDE) {
                robotIndex = i;
                break;
            }
        }
        if (robotIndex == -1) {
            throw std::runtime_error("No site found within Voronoi cell.");
        }

        // Calculate the centroid of the clipped polygon
        //CGAL::Cartesian_converter<K, CGAL::Simple_cartesian<double>> to_double;
        //Point centroid = CGAL::centroid(cell.vertices_begin(), cell.vertices_end(), to_double);
        //mapOfCentroids.emplace(robotIndex, CommonTypes::Vertex(centroid.x(), centroid.y()));

        // Determine the minimum distance between any vertex of the cell and its
        // centroid.  This will be used below to determine the maximum dilation.
        /*
        double minDistance = std::numeric_limits<double>::max();
        for (auto vertex = cell.vertices_begin(); vertex != cell.vertices_end(); ++vertex) {
            double distance = CGAL::squared_distance(*vertex, centroid);
            if (distance < minDistance) {
                minDistance = distance;
            }
        }
        minDistance = std::sqrt(minDistance); // get the actual distance, not the squared distance

        // Choose the set of dilations we will produce.
        std::vector<double> dilations;
        for (double d = -config.robotRadius; d > -minDistance + config.robotRadius; d -= config.dilationDelta) {
            dilations.push_back(d);
        }

        // DEBUG: Add the original polygon to the list of dilated polygons (with a dilation of 0)
        //dilations.push_back(0);
        dilations.clear();
        dilations.push_back(-100);

        //std::cout << "Dilations: ";
        //for (const auto& d : dilations) {
        //    std::cout << d << " ";
        //}
        //std::cout << std::endl;
        */


        // Create all of the inner offset polygons from this cell.
        auto offset_polygons = CGAL::create_interior_skeleton_and_offset_polygons_2(config.dilationDelta, cell);

        // Loop through the offset polygons and create the dilated polygons
        std::vector<DilatedPolygon> dilatedPolygons;
        for (auto poly : offset_polygons) {
            DilatedPolygon dilatedPolygon;
            for (auto v = poly->vertices_begin(); v != poly->vertices_end(); ++v)
                dilatedPolygon.vertices.push_back(CommonTypes::Vertex(v->x(), v->y()));
            dilatedPolygons.push_back(dilatedPolygon);
        }

        if (!dilatedPolygons.empty())
            mapOfDilatedPolygons.emplace(robotIndex, dilatedPolygons);
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

}; // namespace CgalVoronoi