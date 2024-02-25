#pragma once
#include <vector>
#include <map>

struct TrackPoint {
    double x;
    double y;
};

struct Track {
    double score;
    std::vector<TrackPoint> points;
};

// Define a Plan as a mapping from robot indices to vectors of TrackPoints.
using Plan = std::map<int, std::vector<Track>>;