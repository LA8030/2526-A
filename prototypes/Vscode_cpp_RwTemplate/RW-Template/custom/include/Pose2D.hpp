
#pragma once

#include <cmath>

#define POINT2D_INVALID_COORD -9999.0

class Pose2D
{
public:
    Pose2D() : x(POINT2D_INVALID_COORD), y(POINT2D_INVALID_COORD), theta(POINT2D_INVALID_COORD) {}
    ~Pose2D() = default;
    Pose2D(const Pose2D &other) : x(other.x), y(other.y), theta(other.theta) {}
    Pose2D &operator=(const Pose2D &other)
    {
        if (this != &other)
        {
            x = other.x;
            y = other.y;
            theta = other.theta;
        }
        return *this;
    }

    bool isValid() const
    {
        return (x != POINT2D_INVALID_COORD) && (y != POINT2D_INVALID_COORD) && (theta != POINT2D_INVALID_COORD);
    }

    void invalidate() {
        x = POINT2D_INVALID_COORD;
        y = POINT2D_INVALID_COORD;
        theta = POINT2D_INVALID_COORD;
    }

    Pose2D(double fx = 0.0, double fy = 0.0, double ftheta = 0.0) : x(fx), y(fy), theta(ftheta) {}

    double distanceTo(const Pose2D &other) const
    {
        return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
    }

    double angleTo(const Pose2D &other) const
    {
        return std::atan2(other.y - y, other.x - x);
    }

public:
    double x;
    double y;
    double theta; 
};