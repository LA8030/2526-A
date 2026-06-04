
#pragma once

#include "Pose2D.hpp"

class TagMap
{
public:
    TagMap() = default;
    ~TagMap() = default;

    Pose2D whereAmI(int tagID, int pixelX, int pixelY, double tagWidth, double tagHeight, double gyroHeading);

    Pose2D getTagCoordinates(int tagID);
    static double getYawToTag(int tagID);
    static double getDistanceToTag(double tagWidth, double tagHeight);
};