#pragma once
#include <geos/geom/GeometryFactory.h>
#include <geos/triangulate/VoronoiDiagramBuilder.h>
//#include <geos/geom/CoordinateSequenceFactory.h>
#include <geos/geom/CoordinateArraySequence.h>
#include "CommonTypes.hpp"
#include "WorldState.hpp"
#include "WorldConfig.hpp"
#include "Angles.hpp"

using namespace geos::geom;
using namespace geos::triangulate;

namespace GeosVoronoi {

struct DilatedPolygon {
    double dilation;
    std::vector<CommonTypes::Vertex> vertices;
};

// A Skeleton represents the information extracted from a dilated polygon that
// will be used to form a Curve.
/*
struct Skeleton {
    // The robot's pose is the beginning of every Skeleton.
    Pose beginningPose;

    // We'll "unwrap" all of the dilated polygon's vertices in the right order
    // to match the robot's orientation.
    std::vector<CommonTypes::Vertex> vertices;
};
*/

using MapOfVectorOfDilatedPolygons = std::map<size_t, std::vector<DilatedPolygon>>;

//using MapOfVectorOfSkeletons = std::map<size_t, std::vector<Skeleton>>;

using MapOfCentroids = std::map<size_t, CommonTypes::Vertex>;

class GeosVoronoiBuilder {
private:
    Envelope envelope;
    MapOfVectorOfDilatedPolygons mapOfDilatedPolygons;
    //MapOfVectorOfSkeletons mapOfSkeletons;
    MapOfCentroids mapOfCentroids;

    const double siteShiftRate = 0.01;

public:
    GeosVoronoiBuilder() : envelope(0, config.width, 0, config.height)
    {
    }

    void compute(std::shared_ptr<WorldState> worldState) {
        computeDilatedPolygons(worldState);

        // From each dilated polygon we select a skeleton that will be used to form a track.
        //computeSkeletons(worldState);
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

    //MapOfVectorOfSkeletons getMapOfSkeletons() {
    //    return mapOfSkeletons;
    //}

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
                CommonTypes::Vertex centroidVertex = {centroid->getX(), centroid->getY()};
                mapOfCentroids.emplace(robotIndex, centroidVertex);

                // Determine the shortest distance from robotPoint to the boundary of the polygon
                auto boundary = clippedPolygon->getBoundary();
                double robotToPolyDist = robotPoint->distance(boundary.get());
                // std::cout << "robotToPolyDist: " << robotToPolyDist << std::endl;

                // Determine the minimum distance between any vertex of the clipped
                // polygon and the centroid of the polygon.  This will be used to
                // determine the maximum dilation.
                double minDistance = std::numeric_limits<double>::max();
                for (int j = 0; j < clippedPolygon->getNumPoints(); ++j) {
                    auto coord = clippedPolygon->getCoordinates()->getAt(j);
                    double distance = std::sqrt((coord.x - centroid->getX()) * (coord.x - centroid->getX()) + (coord.y - centroid->getY()) * (coord.y - centroid->getY()));
                    if (distance < minDistance) {
                        minDistance = distance;
                    }
                }

                // Choose the set of dilations we will produce.
                std::vector<double> dilations;
                for (double d = -config.robotRadius; d > -minDistance + config.robotRadius; d -= config.dilationDelta) {
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
                    DilatedPolygon dilatedPolygon;
                    dilatedPolygon.dilation = d;

                    //auto geosDilatedPolygon = clippedPolygon->buffer(d);
                    //unique_ptr<geos::geom::Geometry> geosDilatedPolygon = clippedPolygon->buffer(d)->buffer(-50)->buffer(50);
                    
                    cout << "BEFORE geosPolygonProcessing" << endl;
                    int nPoints = clippedPolygon->getNumPoints();
                    cout << "clippedPolygon->getNumPoints(): " << nPoints << endl;
                    auto geosDilatedPolygon = geosPolygonProcessing(factory, clippedPolygon, d, robotPoint, robot);
                    if (geosDilatedPolygon == nullptr) {
                        continue;
                    }
                    nPoints = geosDilatedPolygon->getNumPoints();
                    cout << "geosDilatedPolygon->getNumPoints(): " << nPoints << endl;
                    cout << "AFTER geosPolygonProcessing" << endl;

                    // Note that we are not including the last point, which is the same as the first
                    
                    for (int j = 0; j < nPoints - 1; ++j) {
                        auto coord = geosDilatedPolygon->getCoordinates()->getAt(j);
                        dilatedPolygon.vertices.push_back(CommonTypes::Vertex(coord.x, coord.y));
                    }

                    if (dilatedPolygon.vertices.size() >= 3)
                        dilatedPolygons.push_back(dilatedPolygon);
                }

                // std::cout << "Dilated polygons: " << dilatedPolygons.size() << std::endl;
                
                if (!dilatedPolygons.empty()) {
                    mapOfDilatedPolygons.emplace(robotIndex, dilatedPolygons);
                }
            }
        }
        // std::cout << "END OF COMPUTE DILATED POLYGONS " << std::endl;
    }

    unique_ptr<geos::geom::Geometry> geosPolygonProcessing(GeometryFactory::Ptr &factory, unique_ptr<geos::geom::Geometry> &clippedPolygon, double dilation, geos::geom::Point *robotPoint, Robot &robot) {
        cout << "geosPolygonProcessing: A" << endl;
        //auto geosDilatedPolygon = clippedPolygon->buffer(dilation);
        auto geosDilatedPolygon = clippedPolygon->buffer(dilation)->buffer(-50)->buffer(50);
        if (geosDilatedPolygon->getNumPoints() < 3)
            return nullptr;
        cout << "geosDilatedPolygon->getNumPoints(): " << geosDilatedPolygon->getNumPoints() << endl;


        // Define a point ahead of the robot.
        double aheadDistance = 100;
        double aheadX = robot.x + aheadDistance * std::cos(robot.theta);
        double aheadY = robot.y + aheadDistance * std::sin(robot.theta);
        auto aheadPoint = GeometryFactory::getDefaultInstance()->createPoint({aheadX, aheadY});
        cout << "geosPolygonProcessing: B" << endl;

        // p1 is the closest point on the polygon to the robot's position.
        geos::geom::Point *p1 = findClosestPointOnPolygon(geosDilatedPolygon, robotPoint);
        cout << "geosPolygonProcessing: C" << endl;

        // p2 is the closest point on the polygon to the point ahead of the robot.
        geos::geom::Point *p2 = findClosestPointOnPolygon(geosDilatedPolygon, aheadPoint);
        cout << "geosPolygonProcessing: D" << endl;

        // Determine if the robot is inside the polygon.
        bool inside = geosDilatedPolygon->contains(robotPoint);
        cout << "geosPolygonProcessing: E" << endl;

        if (inside) {
        } else {
            // Create a CoordinateArraySequence using the following points: robotPoint, aheadPoint, p1, p2, and robotPoint.
            
            cout << "geosPolygonProcessing: F" << endl;

            geos::geom::CoordinateArraySequence *cas = new geos::geom::CoordinateArraySequence();

            cas->add(geos::geom::Coordinate(robotPoint->getCoordinate()->x, robotPoint->getCoordinate()->y));
            cas->add(geos::geom::Coordinate(aheadPoint->getCoordinate()->x, aheadPoint->getCoordinate()->y));
            cas->add(geos::geom::Coordinate(p2->getCoordinate()->x, p2->getCoordinate()->y));
            auto centroid = geosDilatedPolygon->getCentroid();
            cas->add(geos::geom::Coordinate(centroid->getX(), centroid->getY()));
            cas->add(geos::geom::Coordinate(p1->getCoordinate()->x, p1->getCoordinate()->y));
            cas->add(geos::geom::Coordinate(robotPoint->getCoordinate()->x, robotPoint->getCoordinate()->y));
            auto linearRing = factory->createLinearRing(cas);

            /*
            std::vector<Coordinate> coordinates;
            Coordinate r{robotPoint->getCoordinate()->x, robotPoint->getCoordinate()->y};
            Coordinate a{aheadPoint->getCoordinate()->x, aheadPoint->getCoordinate()->y};
            Coordinate _p2{p2->getCoordinate()->x, p2->getCoordinate()->y};
            auto centroid = geosDilatedPolygon->getCentroid();
            Coordinate _c{centroid->getX(), centroid->getY()};
            Coordinate _p1{p1->getCoordinate()->x, p1->getCoordinate()->y};
            coordinates.push_back(r);
            coordinates.push_back(a);
            coordinates.push_back(_p2);
            coordinates.push_back(_c);
            coordinates.push_back(_p1);
            coordinates.push_back(r);
            

            cout << "geosPolygonProcessing: G" << endl;
            
            // Create a CoordinateArraySequence from the vector of coordinates
            auto coordinateSequenceFactory = factory->getCoordinateSequenceFactory();
            if (coordinateSequenceFactory == nullptr) {
                cout << "NULL coordinateSequenceFactory" << endl;
            }
            geos::geom::CoordinateSequence::Ptr coordSeq = coordinateSequenceFactory->create(coordinates.size(), 2);
            if (coordSeq == nullptr) {
                cout << "NULL coordSeq" << endl;
            }
            for (int i = 0; i < coordinates.size(); ++i) {
                coordSeq->setAt(coordinates[i], i);
                coordSeq->operator[](
            }
            cout << "geosPolygonProcessing: H" << endl;
            

            // Create a LinearRing from the CoordinateArraySequence
            auto linearRing = factory->createLinearRing(coordSeq);
            */
            if (linearRing == nullptr) {
                cout << "NULL linearRing" << endl;
            }

            cout << "geosPolygonProcessing: I" << endl;
            // Create a Polygon from the LinearRing
            std::unique_ptr<Polygon> polygonPtr(factory->createPolygon(linearRing, nullptr));
            if (polygonPtr == nullptr) {
                cout << "NULL polygonPtr" << endl;
                return nullptr;
            }
            if (!polygonPtr->isValid()) {
                cout << "polygonPtr is NOT valid" << endl;
                return nullptr;
            }

            cout << "polygonPtr->getNumPoints(): " << polygonPtr->getNumPoints() << endl;
            cout << "geosDilatedPolygon->getNumPoints(): " << geosDilatedPolygon->getNumPoints() << endl;
            cout << "polygonPtr->isValid(): " << polygonPtr->isValid() << endl;
            cout << "geosDilatedPolygon->isValid(): " << geosDilatedPolygon->isValid() << endl;

            cout << "geosPolygonProcessing: J" << endl;
            // Compute the union of geosDilatedPolygon and polygonPtr
            auto unionPolygon = geosDilatedPolygon->Union(polygonPtr.get());
            cout << "unionPolygon->getNumPoints(): " << unionPolygon->getNumPoints() << endl;
            cout << "geosPolygonProcessing: K" << endl;
            
            // return unionPolygon;
        }
        cout << "geosPolygonProcessing: L" << endl;
        
        return geosDilatedPolygon;
    }

    geos::geom::Point* findClosestPointOnPolygon(unique_ptr<geos::geom::Geometry> &polygon, geos::geom::Point *point) {
        cout << "findClosestPointOnPolygon START" << endl;
        // Find the closest edge of the polygon to the given point.
        auto closestEdgeIndices = findClosestEdgeToPoint(polygon, point);
        CommonTypes::Vertex first = {polygon->getCoordinates()->getAt(closestEdgeIndices.first).x, polygon->getCoordinates()->getAt(closestEdgeIndices.first).y};
        CommonTypes::Vertex second = {polygon->getCoordinates()->getAt(closestEdgeIndices.second).x, polygon->getCoordinates()->getAt(closestEdgeIndices.second).y};
        cout << "findClosestPointOnPolygon: AA" << endl;
        
        // We find the projection of the point's position onto the closest edge
        CommonTypes::Vertex pointV = {point->getCoordinates()->getAt(0).x, point->getCoordinates()->getAt(0).y};
        CommonTypes::Vertex a = pointV - first;
        CommonTypes::Vertex b = {second.x - first.x, second.y - first.y};
        CommonTypes::Vertex proj = first + b * (a.dot(b) / b.dot(b));
        cout << "findClosestPointOnPolygon: BB" << endl;

        // Convert proj into a geos point.
        auto projPoint = GeometryFactory::getDefaultInstance()->createPoint({proj.x, proj.y});
        cout << "findClosestPointOnPolygon - END" << endl;

        return projPoint;
    }

    std::pair<size_t, size_t> findClosestEdgeToPoint(unique_ptr<geos::geom::Geometry> &polygon, geos::geom::Point *point)
    {
        cout << "findClosestEdgeToPoint START" << endl;
        std::pair<size_t, size_t> closestEdgeIndices = {-1, -1};
        double minDistance = std::numeric_limits<double>::max();
        cout << "polygon->getNumPoints(): " << polygon->getNumPoints() << endl;
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
        cout << "findClosestEdgeToPoint END" << endl;
        return closestEdgeIndices;
    }

    /*
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
                std::pair<CommonTypes::Vertex, CommonTypes::Vertex> closestEdge;
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

                Skeleton skeleton;
                skeleton.beginningPose = robotPose;

                // We'll "unwrap" all of the dilated polygon's vertices in the right order.
                // The first vertex will the one with the smallest relative angle. 
                int firstVertexIndex = 0;
                int indexDelta = 0;
                if (relativeAngle1 < relativeAngle2) {
                    firstVertexIndex = closestEdgeIndices.first;
                    indexDelta = closestEdgeIndices.first - closestEdgeIndices.second;
                } else {
                    firstVertexIndex = closestEdgeIndices.second;
                    indexDelta = closestEdgeIndices.second - closestEdgeIndices.first;
                }

                skeleton.vertices.push_back(vertices[firstVertexIndex]);

                cout << "Dilated polygon vertices: " << vertices.size() << endl;
                for (int k = firstVertexIndex + indexDelta; ; k += indexDelta) {
                    int k_mod = (k + vertices.size()) % vertices.size();
                    if (k_mod == firstVertexIndex) {
                        break;
                    }
                    cout << "k: " << k << endl;
                    cout << "k_mod: " << k_mod << endl;
                    skeleton.vertices.push_back(vertices[k_mod]);
                }
                cout << "Skeleton vertices: " << skeleton.vertices.size() << endl;

                skeletons.push_back(skeleton);
            }
            if (!skeletons.empty()) {
                mapOfSkeletons.emplace(robotIndex, skeletons);
            }
        }
        //std::cout << "END OF COMPUTE SKELETONS" << std::endl;
    }
    */
};

}; // namespace Voronoi