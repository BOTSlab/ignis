#pragma once
#include <vector>
#include <map>
#include <math.h>

namespace CommonTypes {

struct Vec2 {
    double x, y;

    Vec2(double x = 0.0, double y = 0.0)
        : x(x), y(y) {}

    bool operator==(const Vec2 &other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Vec2 &other) const
    {
        return !(*this == other);
    }

    double distance(const Vec2 &other) const
    {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    Vec2 operator+(const Vec2 &other) const
    {
        return Vec2(x + other.x, y + other.y);
    }

    Vec2 operator-(const Vec2 &other) const
    {
        return Vec2(x - other.x, y - other.y);
    }

    Vec2 operator*(double scalar) const
    {
        return Vec2(x * scalar, y * scalar);
    }

    Vec2 operator/(double scalar) const
    {
        return Vec2(x / scalar, y / scalar);
    }

    Vec2 &operator+=(const Vec2 &other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vec2 &operator-=(const Vec2 &other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vec2 &operator*=(double scalar)
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vec2 &operator/=(double scalar)
    {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    double dot(const Vec2 &other) const
    {
        return x * other.x + y * other.y;
    }   

    double cross(const Vec2 &other) const
    {
        return x * other.y - y * other.x;
    }   

    double magnitude() const
    {
        return std::sqrt(x * x + y * y);
    }   

    Vec2 normalize() const
    {
        return *this / magnitude();
    }   

    double angle() const
    {
        return std::atan2(y, x);
    }   

    Vec2 rotate(double angle) const
    {
        double cosAngle = std::cos(angle);
        double sinAngle = std::sin(angle);
        return Vec2(x * cosAngle - y * sinAngle, x * sinAngle + y * cosAngle);
    }   
};

struct DilatedPolygon {
    double dilation;
    std::vector<CommonTypes::Vec2> vertices;
};

struct Pose {
    double x;
    double y;
    double theta;
};

struct Curve {
    // The score after judgment is applied.
    double score;

    std::vector<Pose> poses;
};

using MapOfCurves = std::map<size_t, Curve>;

using MapOfVectorOfCurves = std::map<size_t, std::vector<Curve>>;

}; // namespace CommonTypes