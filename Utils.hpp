#pragma once
#include "CommonTypes.hpp"
using CommonTypes::Vec2;

namespace Utils {

/**
 * Checks if a line segment intersects with a circle.
 *
 * @param segmentStart The starting point of the line segment.
 * @param segmentEnd The ending point of the line segment.
 * @param circleCenter The center of the circle.
 * @param circleRadius The radius of the circle.
 * @return True if the line segment intersects with the circle, false otherwise.
 */
bool segmentIntersectsCircle(Vec2 segmentStart, Vec2 segmentEnd, Vec2 circleCenter, double circleRadius)
{
    // Get the vector from the start of the segment to the circle center.
    Vec2 segmentToCircle = circleCenter - segmentStart;

    // Get the vector representing the segment.
    Vec2 segment = segmentEnd - segmentStart;

    // Get the projection of the segmentToCircle vector onto the segment vector.
    double projection = segmentToCircle.x * segment.x + segmentToCircle.y * segment.y;

    // If the projection is negative, then the circle is behind the segment.
    if (projection < 0)
        return false;

    // Get the squared length of the segment.
    double segmentLengthSquared = segment.x * segment.x + segment.y * segment.y;

    // If the projection is greater than the squared length of the segment, then the circle is beyond the segment.
    if (projection > segmentLengthSquared)
        return false;

    // Get the squared distance from the circle center to the segment.
    double distanceSquared = segmentToCircle.x * segmentToCircle.x + segmentToCircle.y * segmentToCircle.y - projection * projection / segmentLengthSquared;

    // If the squared distance is less than the squared radius, then the segment intersects the circle.
    return distanceSquared < circleRadius * circleRadius;
}

}; // namespace Utils