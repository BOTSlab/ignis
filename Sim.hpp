#include <vector>
#include <cmath>
#include <utility>

struct Circle {
    double x, y, r;
    Circle(double x, double y, double r) : x(x), y(y), r(r) {}
};

class Sim {
public:
    std::vector<std::pair<Circle, Circle>> checkForCircleCircleCollisions(const std::vector<Point>& circles) {

        std::vector<std::pair<Circle, Circle>> collisions;
        for (size_t i = 0; i < circles.size(); ++i) {
            for (size_t j = i + 1; j < circles.size(); ++j) {
                if (check_collision(circles[i], circles[j])) {
                    collisions.push_back(std::make_pair(circles[i], circles[j]));
                }
            }
        }
        return collisions;
    }

private:
    bool check_collision(const Circle& circle1, const Circle& circle2) {
        double distance = std::sqrt(std::pow(circle1.x - circle2.x, 2) + std::pow(circle1.y - circle2.y, 2));
        return distance <= (circle1.r + circle2.r);
    }
};

// Usage:
// std::vector<Circle> circles = {Circle(0, 0, 1), Circle(1, 1, 1), Circle(3, 3, 1)};
// Sim sim;
// auto collisions = sim.step(circles);
// for (const auto& collision : collisions) {
//     std::cout << "Collision between circle at (" << collision.first.x << ", " << collision.first.y << ") and circle at (" << collision.second.x << ", " << collision.second.y << ")\n";
// }