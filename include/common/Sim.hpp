#pragma once
#include <vector>
#include "common/WorldState.hpp"
#include "common/Config.hpp"

// Explicitly making this a namespace, not a class to emphasize that there is
// no "simulator" which maintains state.
namespace Sim {

const double dragCoefficient = 0.9;

// Handle collisions between all circle bodies in the world by updating their velocities.
void dynamicCollisionHandling(std::vector<CircleBody*> allBodies, int &nRobotRobotCollisions, int &nRobotPuckCollisions)
{
    Config &config = Config::getInstance();

    for (auto& b1 : allBodies) {
        for (auto& b2 : allBodies) {
            if (b1 == b2)
                continue;

            double distance = std::sqrt(std::pow(b1->pos.x - b2->pos.x, 2) + std::pow(b1->pos.y - b2->pos.y, 2));
            double sumOfRadii = b1->radius + b2->radius;
            if (distance < sumOfRadii) {
                // Calculate the normal vector between the centers of the circles
                double nx = (b1->pos.x - b2->pos.x) / distance;
                double ny = (b1->pos.y - b2->pos.y) / distance;

                // Calculate the relative velocity between the circles
                double relativeVx = b1->vel.x - b2->vel.x;
                double relativeVy = b1->vel.y - b2->vel.y;

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
                b1->vel.x += impulseX / b1->mass;
                b1->vel.y += impulseY / b1->mass;
                b2->vel.x -= impulseX / b2->mass;
                b2->vel.y -= impulseY / b2->mass;

                if (b1->type == BodyType::Robot && b2->type == BodyType::Robot) {
                    nRobotRobotCollisions++;
                    b1->slowedCounter = config.slowedSteps;
                    b2->slowedCounter = config.slowedSteps;
                }

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
    Config &config = Config::getInstance();

    for (auto& b1 : allBodies) {
        for (auto& b2 : allBodies) {
            if (b1 == b2)
                continue;

            double distance = std::sqrt(std::pow(b1->pos.x - b2->pos.x, 2) + std::pow(b1->pos.y - b2->pos.y, 2));
            double sumOfRadii = b1->radius + b2->radius;
            if (distance < sumOfRadii) {
                // Calculate the normal vector between the centers of the circles
                double nx = (b1->pos.x - b2->pos.x) / distance;
                double ny = (b1->pos.y - b2->pos.y) / distance;

                // Calculate the penetration depth
                double penetrationDepth = sumOfRadii - distance;

                // Move the circles away from each other based on the penetration depth
                b1->pos.x += penetrationDepth * nx * 0.5;
                b1->pos.y += penetrationDepth * ny * 0.5;
                b2->pos.x -= penetrationDepth * nx * 0.5;
                b2->pos.y -= penetrationDepth * ny * 0.5;

                if (b1->type == BodyType::Robot && b2->type == BodyType::Robot) {
                    nRobotRobotCollisions++;
                    b1->slowedCounter = config.slowedSteps;
                    b2->slowedCounter = config.slowedSteps;
                }

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
    Config &config = Config::getInstance();

    for (auto& body : allBodies) {
        // Check if the body is outside the boundary
        if (body->pos.x - body->radius < 0) {
            body->pos.x = body->radius;
            body->vel.x *= -1; // Reverse the velocity component in x direction
            if (body->type == BodyType::Robot)
                nRobotBoundaryCollisions++;
        }
        if (body->pos.x + body->radius > config.width) {
            body->pos.x = config.width - body->radius;
            body->vel.x *= -1; // Reverse the velocity component in x direction
            if (body->type == BodyType::Robot)
                nRobotBoundaryCollisions++;
        }
        if (body->pos.y - body->radius < 0) {
            body->pos.y = body->radius;
            body->vel.y *= -1; // Reverse the velocity component in y direction
            if (body->type == BodyType::Robot)
                nRobotBoundaryCollisions++;
        }
        if (body->pos.y + body->radius > config.height) {
            body->pos.y = config.height - body->radius;
            body->vel.y *= -1; // Reverse the velocity component in y direction
            if (body->type == BodyType::Robot)
                nRobotBoundaryCollisions++;
        }
    }
}

void update(std::shared_ptr<WorldState> worldState)
{
    Config &config = Config::getInstance();

    auto allBodies = worldState->getAllCircleBodies();

    dynamicCollisionHandling(allBodies, worldState->nRobotRobotCollisions, worldState->nRobotPuckCollisions);
    staticCollisionHandling(allBodies, worldState->nRobotRobotCollisions, worldState->nRobotPuckCollisions);
    boundaryCollisionHandling(allBodies, worldState->nRobotBoundaryCollisions);

    for (auto& robot : worldState->robots) {
        // Incorporate the control input into the robot's state, but only if the robot is not currently slowed.
        if (robot.slowedCounter == 0) {
            // First make sure the control input is within the allowed range.
            double forward = robot.controlInput.forwardSpeed;
            double angular = robot.controlInput.angularSpeed;
            if (forward > config.maxForwardSpeed)
                forward = config.maxForwardSpeed;
            if (forward < -config.maxForwardSpeed)
                forward = -config.maxForwardSpeed;
            if (angular > config.maxAngularSpeed)
                angular = config.maxAngularSpeed;
            if (angular < -config.maxAngularSpeed)
                angular = -config.maxAngularSpeed;

            robot.vel.x += forward * cos(robot.theta);
            robot.vel.y += forward * sin(robot.theta);
            robot.theta += angular;
        } else {
            robot.slowedCounter--;
        }

        robot.pos.x += robot.vel.x;
        robot.pos.y += robot.vel.y;
        robot.vel.x *= dragCoefficient;
        robot.vel.y *= dragCoefficient;
    }

    for (auto& puck : worldState->pucks) {
        puck.pos.x += puck.vel.x;
        puck.pos.y += puck.vel.y;
        puck.vel.x *= dragCoefficient;
        puck.vel.y *= dragCoefficient;
    }
}

}; // namespace Sim