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

    double length() const
    {
        return std::sqrt(x * x + y * y);
    }   

    Vec2 normalize() const
    {
        return *this / length();
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

enum class BodyType
{
    Robot,
    Puck
};

struct CircleBody
{
    BodyType type;
    Vec2 pos;
    double radius, mass;
    Vec2 vel;

    CircleBody(BodyType type, double x, double y, double radius, double mass, double vx = 0.0, double vy = 0.0)
        : type(type), pos(x, y), radius(radius), mass(mass), vel(vx, vy) {}
};

struct ControlInput
{
    double forwardSpeed, angularSpeed;

    ControlInput(double forwardSpeed = 0.0, double angularSpeed = 0.0)
        : forwardSpeed(forwardSpeed), angularSpeed(angularSpeed) {}
};

struct Robot : public CircleBody
{
    Robot(double x, double y, double radius, double mass, double theta, double vx = 0.0, double vy = 0.0)
        : CircleBody(BodyType::Robot, x, y, radius, mass, vx, vy), theta(theta), controlInput(), siteX(x), siteY(y) {}

    double theta;
    ControlInput controlInput;

    // Represents the site for this robot's Voronoi cell.
    double siteX, siteY;
};

}; // namespace CommonTypes