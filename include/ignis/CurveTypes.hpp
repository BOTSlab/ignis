#pragma once
#include <vector>
#include <map>
#include <math.h>
#include "common/CommonTypes.hpp"

using namespace CommonTypes;

namespace CurveTypes {

struct DilatedPolygon {
    double dilation;
    std::vector<CommonTypes::Vec2> vertices;

    /**
     * Checks if the curve contains the given position.
     *
     * @param position The position to check.
     * @return True if the curve contains the position, false otherwise.
     */
    bool contains(const CommonTypes::Vec2& position) const {
        int i, j, nvert = vertices.size();
        bool c = false;
        for(i = 0, j = nvert - 1; i < nvert; j = i++) {
            if( ((vertices[i].y >= position.y ) != (vertices[j].y >= position.y)) &&
            (position.x <= (vertices[j].x - vertices[i].x) * (position.y - vertices[i].y) / (vertices[j].y - vertices[i].y) + vertices[i].x) )
            c = !c;
        }
        return c;
    }
};

struct Pose {
    double x;
    double y;
    double theta;
};

struct CurvePoint {
    Pose pose;
    double score = 0;
};

class Curve {
private:
    int indexToSeek = 0;

public:
    std::vector<CurvePoint> points;

    double getTotalScore() const {
        double total = 0;
        for (const CurvePoint &p : points)
            total += p.score;
        return total;
    }

    bool isFinishedFollowing() const {
        return indexToSeek == points.size();
    }

    void advanceIndexToSeek() {
        indexToSeek++;
    }

    void setIndexToSeek(int i) {
        indexToSeek = i;
    }

    int getIndexToSeek() const {
        return indexToSeek;
    }
};

using MapOfCurves = std::map<size_t, Curve>;

using MapOfVectorOfCurves = std::map<size_t, std::vector<Curve>>;

struct CurveSensorPosition {
    double forwardOffset, lateralOffset;
};

struct CurveSensorReading {
    CurveSensorPosition position;
    bool value;
};

// A map from robot index to it's sensor reading.
using MapOfCurveSensorReadings = std::map<size_t, std::vector<CurveSensorReading>>;

}; // namespace CurveTypes