#pragma once

namespace CommonTypes {

struct Vertex {
    double x, y;

    Vertex(double x = 0.0, double y = 0.0)
        : x(x), y(y) {}

    bool operator==(const Vertex &other) const
    {
        return x == other.x && y == other.y;
    }

    bool operator!=(const Vertex &other) const
    {
        return !(*this == other);
    }

    double distance(const Vertex &other) const
    {
        double dx = x - other.x;
        double dy = y - other.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    Vertex operator+(const Vertex &other) const
    {
        return Vertex(x + other.x, y + other.y);
    }

    Vertex operator-(const Vertex &other) const
    {
        return Vertex(x - other.x, y - other.y);
    }

    Vertex operator*(double scalar) const
    {
        return Vertex(x * scalar, y * scalar);
    }

    Vertex operator/(double scalar) const
    {
        return Vertex(x / scalar, y / scalar);
    }

    Vertex &operator+=(const Vertex &other)
    {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vertex &operator-=(const Vertex &other)
    {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vertex &operator*=(double scalar)
    {
        x *= scalar;
        y *= scalar;
        return *this;
    }

    Vertex &operator/=(double scalar)
    {
        x /= scalar;
        y /= scalar;
        return *this;
    }

    double dot(const Vertex &other) const
    {
        return x * other.x + y * other.y;
    }   

    double cross(const Vertex &other) const
    {
        return x * other.y - y * other.x;
    }   

    double magnitude() const
    {
        return std::sqrt(x * x + y * y);
    }   

    Vertex normalize() const
    {
        return *this / magnitude();
    }   

    double angle() const
    {
        return std::atan2(y, x);
    }   

    Vertex rotate(double angle) const
    {
        double cosAngle = std::cos(angle);
        double sinAngle = std::sin(angle);
        return Vertex(x * cosAngle - y * sinAngle, x * sinAngle + y * cosAngle);
    }   
};

}; // namespace CommonTypes