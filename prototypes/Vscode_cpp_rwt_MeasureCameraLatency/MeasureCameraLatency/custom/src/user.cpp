#include "vex.h"
#include "motor-control.h"

#include "../custom/include/autonomous.h"
#include "../custom/include/robot-config.h"

#include "../include/Logger.hpp"
#include "../include/Pose2D.hpp"

Logger myLogger;

/*******************************************************/
// Test settings
const int tagIDToTrack = 1; // Specify only one tag so data is more consistent
const int testSpeed =
    60; // Base speed for the test (0-127, where 127 is full voltage)

/*******************************************************/
// Local variables
double initialHeadingToTag = 0.0;
double gyro_bias_at_rest = 0.0;

double getAngleToTag(int tagID)
{
  double retVal = Pose2D::POSE2D_INVALID;

  // Take a snapshot filtering specifically for AprilTags
  aiCam.takeSnapshot(aivision::ALL_TAGS, 8);

  // Check if anything was caught in the frame
  if (aiCam.objectCount > 0)
  {
    for (int i = 0; i < aiCam.objectCount; i++)
    {
      // Max detection range 24" so generally don't expect more than one tag in view at a time, but possible
      vex::aivision::object tag = aiCam.objects[i];

      // Double check that this specific slot contains valid data
      if (tag.exists && (tag.id == tagID))
      {
        retVal = tag.angle;
        break;
      }
    }
  }
  return retVal;
}

void saveDataToLog(std::string testName)
{
  double currentHeading = inertial_sensor.angle(); // 0 to 360 degrees
  double gyroHeadingToTag = currentHeading - initialHeadingToTag;

  double angularVelocity = inertial_sensor.gyroRate(zaxis, dps);

  double tag_angle = getAngleToTag(tagIDToTrack);

  if (tag_angle == Pose2D::POSE2D_INVALID)
  {
    myLogger.textToControllerLcd("Tag not found");
  }
  else
  {
    myLogger.textToControllerLcd(" ");
    myLogger.writeLog(testName + ", " + myLogger.getTimestamp() + "," +
                      myLogger.to_string(angularVelocity) + "," +
                      myLogger.to_string(currentHeading) + "," +
                      myLogger.to_string(gyroHeadingToTag) + "," +
                      myLogger.to_string(tag_angle));
  }
}

bool isImuValid()
{
  if (inertial_sensor.isCalibrating())
  {
    return false;
  }
  if (!inertial_sensor.installed())
  {
    return false;
  }
  return true;
}

std::string prepareForTest()
{
  bool isReadyToTest = false;
  std::string errRet = " ";

  double tagHeading = getAngleToTag(tagIDToTrack);
  if (tagHeading == Pose2D::POSE2D_INVALID)
  {
    errRet = "Tag not found. Make sure tag is visible to camera. ";
  }
  else
  {
    if (!isImuValid())
    {
      errRet = "IMU not valid. Check connections and calibration.";
    }
    else
    {
      double gyroHeading = inertial_sensor.rotation();
      initialHeadingToTag = gyroHeading - tagHeading; // Calculate the
                                                      // initial heading
                                                      // to the tag
      isReadyToTest = true;
    }
  }

  if (isReadyToTest)
  {
    errRet = "READY TO TEST";
  }

  return errRet;
}

// Modify autonomous, driver, or pre-auton code below

void runAutonomous()
{
  int auton_selected = 3;
  switch (auton_selected)
  {
  case 1:
    exampleAuton();
    break;
  case 2:
    exampleAuton2();
    break;
  case 3:
    redGoalRush();
    break;
  case 4:
    break;
  case 5:
    break;
  case 6:
    break;
  case 7:
    break;
  case 8:
    break;
  case 9:
    break;
  }
}

// controller_1 input variables (snake_case)
int ch1, ch2, ch3, ch4;
bool l1, l2, r1, r2;
bool button_a, button_b, button_x, button_y;
bool button_up_arrow, button_down_arrow, button_left_arrow, button_right_arrow;
int chassis_flag = 0;

void runDriver()
{
  stopChassis(coast);
  heading_correction = false;
  while (true)
  {
    // [-100, 100] for controller stick axis values
    ch1 = controller_1.Axis1.value();
    ch2 = controller_1.Axis2.value();
    ch3 = controller_1.Axis3.value();
    ch4 = controller_1.Axis4.value();

    // true/false for controller button presses
    l1 = controller_1.ButtonL1.pressing();
    l2 = controller_1.ButtonL2.pressing();
    r1 = controller_1.ButtonR1.pressing();
    r2 = controller_1.ButtonR2.pressing();
    button_a = controller_1.ButtonA.pressing();
    button_b = controller_1.ButtonB.pressing();
    button_x = controller_1.ButtonX.pressing();
    button_y = controller_1.ButtonY.pressing();
    button_up_arrow = controller_1.ButtonUp.pressing();
    button_down_arrow = controller_1.ButtonDown.pressing();
    button_left_arrow = controller_1.ButtonLeft.pressing();
    button_right_arrow = controller_1.ButtonRight.pressing();

    static bool prev_button_a = false;
    static bool prev_button_b = false;
    static bool prev_button_x = false;
    static bool prev_button_y = false;

    bool just_released_button_A = (prev_button_a && (!button_a));
    prev_button_a = button_a;
    bool just_released_button_B = (prev_button_b && (!button_b));
    prev_button_b = button_b;
    bool just_released_button_X = (prev_button_x && (!button_x));
    prev_button_x = button_x;
    bool just_released_button_Y = (prev_button_y && (!button_y));
    prev_button_y = button_y;

    if (just_released_button_X)
    {

      // Button X was just released
      myLogger.dumpLogToTerminal(); // Send log data to the terminal window
      // (only works if connected by USB)
    }
    else if (just_released_button_Y)
    {

      // Button Y was just released
      myLogger.resetLog(); // Clear the log data after saving to SD card
    }
    else if (just_released_button_A)
    {

      // Button A was just released — Prepare for test
      std::string errMsg = prepareForTest();
      myLogger.writeLog(errMsg);
      myLogger.textToControllerLcd(errMsg);
    }
    else if (just_released_button_B)
    {

      // Button B was just released - End of test
      stopChassis(hold);

      myLogger.writeLog("TEST END");
      myLogger.textToControllerLcd("TEST ENDED");
    }
    else if (button_b)
    {

      // Holding B button down - Spin Left at set speed
      driveChassis(-testSpeed, testSpeed);
    }
    else
    {
      // Arcade control scheme, but with both on the right because my left joystick is sticky
      driveChassis((-ch2 + ch1) * 0.12, (-ch2 - ch1) * 0.12);

      if (ch1 > 5)
      {
        saveDataToLog("SPIN LEFT");
      }
    }

    wait(10, msec);
  }
}

void runPreAutonomous()
{
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // Calibrate inertial sensor
  inertial_sensor.calibrate();

  // Wait for the Inertial Sensor to calibrate
  while (inertial_sensor.isCalibrating())
  {
    wait(10, msec);
  }

  gyro_bias_at_rest = inertial_sensor.heading();

  double current_heading = inertial_sensor.heading();
  Brain.Screen.print(current_heading);

  // odom tracking
  resetChassis();
  if (using_horizontal_tracker && using_vertical_tracker)
  {
    thread odom = thread(trackXYOdomWheel);
  }
  else if (using_horizontal_tracker)
  {
    thread odom = thread(trackXOdomWheel);
  }
  else if (using_vertical_tracker)
  {
    thread odom = thread(trackYOdomWheel);
  }
  else
  {
    thread odom = thread(trackNoOdomWheel);
  }
}