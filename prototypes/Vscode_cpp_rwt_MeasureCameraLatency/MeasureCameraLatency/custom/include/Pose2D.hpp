
#pragma once

class Pose2D
{
public:
    static constexpr int POSE2D_INVALID = -9999;

    Pose2D() : x(POSE2D_INVALID), y(POSE2D_INVALID), theta(POSE2D_INVALID) {}
    ~Pose2D() = default;

    Pose2D(double fx = 0.0, double fy = 0.0, double ftheta = 0.0) : x(fx), y(fy), theta(ftheta) {}
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

    bool isValid() const;
    void invalidate();
    double distanceTo(const Pose2D &other) const;
    double angleTo(const Pose2D &other) const;

public:
    double x;
    double y;
    double theta; 
};