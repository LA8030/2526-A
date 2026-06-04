
#include <math.h>

#include "../custom/include/Pose2D.hpp"
#include "../custom/include/TagMap.hpp"

const Pose2D OVERRIDE_TAG_MAP[]{
    {0.0, 0.0, 0.0},    // Index 0: Dummy/Invalid slot (Tags start at 1)
    {12.0, 72.0, 0.0},  // Index 1: Coordinates for Tag ID 1
    {24.0, 144.0, 0.0}, // Index 2: Coordinates for Tag ID 2
    {72.0, 12.0, 0.0},  // Index 3: Coordinates for Tag ID 3
    {144.0, 72.0, 0.0}, // Index 4: Coordinates for Tag ID 4
    {88.5, 92.0, 0.0}   // Index 5: Coordinates for Tag ID 5 (etc.)
};

// Given a tagID and it's observation info from the point of view of the robot, calculate the robot's position on the field
Pose2D TagMap::whereAmI(int tagID, int pixelX, int pixelY, double tagWidth, double tagHeight, double gyroHeading)
{
    // Get the tag's known field coordinates
    Pose2D tagFieldCoords = getTagCoordinates(tagID);

    // Calculate the angle and distance to the tag using the camera data
    double angleToTag = getYawToTag(pixelX);
    double distanceToTag = getDistanceToTag(tagWidth, tagHeight);

    // Convert angle to radians for calculation
    double angleToTagRad = angleToTag * M_PI / 180.0;

    // Calculate the robot's position relative to the tag
    double robotX = tagFieldCoords.x - distanceToTag * cos(angleToTagRad);
    double robotY = tagFieldCoords.y - distanceToTag * sin(angleToTagRad);

    Pose2D retVal(robotX, robotY, gyroHeading);
    return retVal; // Return the calculated position as a Pose2D object
}

Pose2D TagMap::getTagCoordinates(int tagID)
{
    Pose2D retVal = Pose2D(0, 0, 0);
    retVal.invalidate();

    if ((tagID >= 1) &&
        (tagID < sizeof(OVERRIDE_TAG_MAP) / sizeof(Pose2D)))
    {
        retVal = OVERRIDE_TAG_MAP[tagID];
    }
    return retVal;
}

double TagMap::getYawToTag(int pixelX)
{
    // For a pinhole camera model:
    //   angle = atan2(pixel_offset, focal_length_in_pixels)
    //
    // focal_length_pixels = (image_width / 2) / tan(hFOV / 2)
    // With hFOV = 74.0 deg: focal_length_pixels = 160 / tan(37.0 deg) ≈ 228.5

    const double HFOV_DEG =
        74.0; // Source here:
              // https://kb.vex.com/hc/en-us/articles/24173352365972-Comparing-the-AI-Vision-Sensor-to-the-V5-Vision-Sensor
    const int IMAGE_WIDTH = 320;

    double focalLengthPixels =
        (IMAGE_WIDTH / 2.0) / tan(HFOV_DEG * M_PI / 360.0);

    double pixel_offset = pixelX - (IMAGE_WIDTH / 2.0); // signed, pixels from center

    // Returns degrees, positive = tag is to the right of center
    return atan2(pixel_offset, focalLengthPixels) * 180.0 / M_PI;
}

double TagMap::getDistanceToTag(double tagWidth, double tagHeight)
{
    // For a pinhole camera model:
    //   distance = (real_world_size * focal_length_in_pixels) / observed_size_in_pixels
    //
    // We can calculate this for both width and height and then average for better accuracy

    const double HFOV_DEG =
        74.0; // Source here:
              // https://kb.vex.com/hc/en-us/articles/24173352365972-Comparing-the-AI-Vision-Sensor-to-the-V5-Vision-Sensor
    const int IMAGE_WIDTH = 320;

    double focalLengthPixels =
        (IMAGE_WIDTH / 2.0) / tan(HFOV_DEG * M_PI / 360.0);

    double distanceByWidth =
        (tagWidth * focalLengthPixels) / tagWidth; // tagWidth in pixels from camera data
    double distanceByHeight =
        (tagHeight * focalLengthPixels) / tagHeight; // tagHeight in pixels from camera data

    return (distanceByWidth + distanceByHeight) / 2.0;
}