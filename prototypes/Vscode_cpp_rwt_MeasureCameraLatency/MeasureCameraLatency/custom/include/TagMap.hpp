
#pragma once

#include "Pose2D.hpp"

class TagMap
{
public:
    TagMap() = default;
    ~TagMap() = default;

    Pose2D whereAmI(int tagID, int pixelX, int pixelY, double tagWidth, double tagHeight, double gyroHeading);

private:
    Pose2D getTagCoordinates(int tagID);
    double getAngleToTag(int pixelX);
    double getDistanceToTag(double tagWidth, double tagHeight);
};