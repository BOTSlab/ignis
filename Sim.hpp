#pragma once
#include <cmath>
#include <utility>
#include <vector>
#include "WorldState.hpp"
#include "WorldConfig.hpp"

// Explicitly making this a namespace, not a class to emphasize that there is
// no "simulator" which maintains state.
namespace Sim {

const double dragCoefficient = 0.9;

// Handle collisions between all circle bodies in the world by updating their velocities.
void dynamicCollisionHandling(std::vector<CircleBody*> allBodies, int &nRobotRobotCollisions, int &nRobotPuckCollisions)
{
    for (auto& b1 : allBodies) {
        for (auto& b2 : allBodies) {
            if (b1 == b2)
                continue;

            double distance = std::sqrt(std::pow(b1->x - b2->x, 2) + std::pow(b1->y - b2->y, 2));
            double sumOfRadii = b1->radius + b2->radius;
            if (distance < sumOfRadii) {
                // Calculate the normal vector between the centers of the circles
                double nx = (b1->x - b2->x) / distance;
                double ny = (b1->y - b2->y) / distance;

                // Calculate the relative velocity between the circles
                double relativeVx = b1->vx - b2->vx;
                double relativeVy = b1->vy - b2->vy;

                // Calculate the relative velocity in the normal direction
                double relativeVelocity = relativeVx * nx + relativeVy * ny;

                // If the circles are moving away from each other, no collision response is needed
                if (relativeVelocity > 0) {
                    continue;
                }

                // Calculate the impulse magnitude
                double impulseMagnitude = -(1 + dragCoefficient) * relativeVelocity;
                impulseMagnitude /= (1 / b2->mass + 1 / b1->mass);

                // Apply the impulse to the circles
                double impulseX = impulseMagnitude * nx;
                double impulseY = impulseMagnitude * ny;
                b1->vx += impulseX / b1->mass;
                b1->vy += impulseY / b1->mass;
                b2->vx -= impulseX / b2->mass;
                b2->vy -= impulseY / b2->mass;

                if (b1->type == BodyType::Robot && b2->type == BodyType::Robot)
                    nRobotRobotCollisions++;

                if ((b1->type == BodyType::Robot && b2->type == BodyType::Puck) |
                    (b1->type == BodyType::Puck && b2->type == BodyType::Robot))
                    nRobotPuckCollisions++;
            }
        }
    }
}

// Handle collisions between all circle bodies by modifying their positions.
void staticCollisionHandling(std::vector<CircleBody*> allBodies, int &nRobotRobotCollisions, int &nRobotPuckCollisions)
{
    for (auto& b1 : allBodies) {
        for (auto& b2 : allBodies) {
            if (b1 == b2)
                continue;

            double distance = std::sqrt(std::pow(b1->x - b2->x, 2) + std::pow(b1->y - b2->y, 2));
            double sumOfRadii = b1->radius + b2->radius;
            if (distance < sumOfRadii) {
                // Calculate the normal vector between the centers of the circles
                double nx = (b1->x - b2->x) / distance;
                double ny = (b1->y - b2->y) / distance;

                // Calculate the penetration depth
                double penetrationDepth = sumOfRadii - distance;

                // Move the circles away from each other based on the penetration depth
                b1->x += penetrationDepth * nx * 0.5;
                b1->y += penetrationDepth * ny * 0.5;
                b2->x -= penetrationDepth * nx * 0.5;
                b2->y -= penetrationDepth * ny * 0.5;

                if (b1->type == BodyType::Robot && b2->type == BodyType::Robot)
                    nRobotRobotCollisions++;

                if ((b1->type == BodyType::Robot && b2->type == BodyType::Puck) |
                    (b1->type == BodyType::Puck && b2->type == BodyType::Robot))
                    nRobotPuckCollisions++;
            }
        }
    }
}

// Handle collisions between all circle bodies and the boundaries of the world.
void boundaryCollisionHandling(std::vector<CircleBody*> allBodies, int &nRobotBoundaryCollisions)
{
    for (auto& body : allBodies) {
        // Check if the body is outside the boundary
        if (body->x - body->radius < 0) {
            body->x = body->radius;
            body->vx *= -1; // Reverse the velocity component in x direction
            if (body->type == BodyType::Robot)
                nRobotBoundaryCollisions++;
        }
        if (body->x + body->radius > config.width) {
            body->x = config.width - body->radius;
            body->vx *= -1; // Reverse the velocity component in x direction
            if (body->type == BodyType::Robot)
                nRobotBoundaryCollisions++;
        }
        if (body->y - body->radius < 0) {
            body->y = body->radius;
            body->vy *= -1; // Reverse the velocity component in y direction
            if (body->type == BodyType::Robot)
                nRobotBoundaryCollisions++;
        }
        if (body->y + body->radius > config.height) {
            body->y = config.height - body->radius;
            body->vy *= -1; // Reverse the velocity component in y direction
            if (body->type == BodyType::Robot)
                nRobotBoundaryCollisions++;
        }
    }
}

void update(std::shared_ptr<WorldState> worldState)
{
    auto allBodies = worldState->getAllCircleBodies();

    dynamicCollisionHandling(allBodies, worldState->nRobotRobotCollisions, worldState->nRobotPuckCollisions);
    staticCollisionHandling(allBodies, worldState->nRobotRobotCollisions, worldState->nRobotPuckCollisions);
    boundaryCollisionHandling(allBodies, worldState->nRobotBoundaryCollisions);

    for (auto& robot : worldState->robots) {
        // Incorporate the control input into the robot's state.
        robot.vx += robot.controlInput.forwardSpeed * cos(robot.theta);
        robot.vy += robot.controlInput.forwardSpeed * sin(robot.theta);
        robot.theta += robot.controlInput.angularSpeed;

        robot.x += robot.vx;
        robot.y += robot.vy;
        robot.vx *= dragCoefficient;
        robot.vy *= dragCoefficient;
    }

    for (auto& puck : worldState->pucks) {
        puck.x += puck.vx;
        puck.y += puck.vy;
        puck.vx *= dragCoefficient;
        puck.vy *= dragCoefficient;
    }
}

}; // namespace Sim