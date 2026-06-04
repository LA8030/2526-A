#include <cmath>
#include "../custom/include/Pose2D.hpp"

bool Pose2D::isValid() const
{
    return (x != POSE2D_INVALID) && (y != POSE2D_INVALID) && (theta != POSE2D_INVALID);
}

void Pose2D::invalidate() {
    x = POSE2D_INVALID;
    y = POSE2D_INVALID;
    theta = POSE2D_INVALID;
}

double Pose2D::distanceTo(const Pose2D &other) const
{
    return std::sqrt(std::pow(other.x - x, 2) + std::pow(other.y - y, 2));
}

double Pose2D::angleTo(const Pose2D &other) const
{
    return std::atan2(other.y - y, other.x - x);
}
