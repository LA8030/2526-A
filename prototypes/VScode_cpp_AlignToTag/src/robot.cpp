#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// VEXcode device constructors
motor leftMotor = motor(PORT1, ratio18_1, false);
motor rightMotor = motor(PORT10, ratio18_1, true);

gyro myGyro = gyro(Brain.ThreeWirePort.D);
smartdrive Drivetrain = smartdrive(leftMotor, rightMotor,
                                   myGyro, 319.19, 320, 130, mm, 1);

motor clawMotor = motor(PORT3, ratio18_1, false);
motor armMotor = motor(PORT8, ratio18_1, false);

vex::aivision AIVision5(PORT5, aivision::ALL_TAGS);

controller Controller1 = controller(primary);

// VEXcode generated functions

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void)
{
  Brain.Screen.print("Device initialization...");
  Brain.Screen.setCursor(2, 1);
  // calibrate the drivetrain gyro
  wait(200, msec);
  myGyro.startCalibration(1);
  Brain.Screen.print("Calibrating Gyro for Drivetrain");
  // wait for the gyro calibration process to finish
  while (myGyro.isCalibrating())
  {
    wait(25, msec);
  }
  // reset the screen now that the calibration is complete
  Brain.Screen.clearScreen();
  Brain.Screen.setCursor(1, 1);
  wait(50, msec);
  Brain.Screen.clearScreen();

  // Set stopping behavior so the arm and claw stay in place when idle
  armMotor.setStopping(hold);
  clawMotor.setStopping(hold);
}

void update50hz()
{
  // For Arcade drive, Right Joystick (Axis 2) controls forward/backward, Right Joystick (Axis 1) turns
  int drivePower = Controller1.Axis2.position(); // Range -100 to +100
  int turnPower = Controller1.Axis1.position();

  bool isDrivingForward = (drivePower > 0);

  if (isDrivingForward && Controller1.ButtonA.pressing())
  {
    // Intended functionality:
    // Operator drives toward goal holding button "A"
    // Normal Arcade drive while tag ID 1 is not visible (Left joystick Fwd/Rev.  Right Joystick Left/Right)
    // When tag ID 1 becomes visible it will auto-steer to the tag and stop when it gets there
    // Drive releases button "A" to restore normal Arcade drive

    // If driving forward AND holding "A" button AND an AprilTag is visible
    //   If close to tag
    //     STOP
    //   Else
    //     Ignore R/L joystick and steer toward the tag
    // Else
    //   Regular Arcade drive

    AIVision5.takeSnapshot(vex::aivision::ALL_TAGS);
    printf("Object count = %d\n", (int)AIVision5.objectCount);

    // Check if any tags were found in the frame
    if (AIVision5.objectCount > 0)
    {
      // Check if any tags were found in the frame
      if (AIVision5.objectCount > 0)
      {

        // Loop through detected objects to look for Tag ID 1
        for (int i = 0; i < AIVision5.objectCount; i++)
        {
          if (AIVision5.objects[i].id == 1)
          {

            // Pull screen tracking data
            int centerX = AIVision5.objects[i].centerX; // Center X (0 to 320)
            int centerY = AIVision5.objects[i].centerY; // Center Y (0 to 240)
            int width = AIVision5.objects[i].width;     // Used to estimate distance

            // Print detection data to the VEXcode Terminal
            printf("Found Tag 1! Screen X: %d, Screen Y: %d, Width: %d\n", centerX, centerY, width);

            bool isCloseToTag = (width > 110);
            if (isCloseToTag)
            {
              // Close to tag -> STOP
              drivePower = 0.0;
              turnPower = 0.0;
            }
            else
            {
              int errorX = (100 * (centerX - 160)) / 160; // Range -100 far left to +100 far right
              double Pcontrol = 0.4;                      // Constant value in range 0.0 to 1.0, determines how strongly the correction is applied

              // PROPORTIONAL control means more error --> stronger correction.
              // Like a spring pulling towrd the desired path
              turnPower = Pcontrol * errorX;
              printf(" errorX= %d, Pcontrol*err= %d\n", errorX, turnPower);
            }
            break;  // Stop looking once Tag ID 1 is found
          }
        }
      }
    }
  }
  printf("Power FR: %d   LR: %d\n", drivePower, turnPower);

  Drivetrain.arcade(drivePower, turnPower);
}
