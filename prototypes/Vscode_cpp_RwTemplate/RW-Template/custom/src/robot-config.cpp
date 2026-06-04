#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller controller_1 = controller(primary);

// IMPORTANT: Remember to modify the example motors according to the guide. 
// Also remember to add respective device declarations to custom/include/robot-config.h
// Format: motor(port, gearSetting, reversed)
// gearSetting is one of the following: ratio36_1(red), ratio18_1(green), ratio6_1(blue)
// all chassis motors should be reversed appropriately so that they spin vertical when given a positive voltage input
// such as driveChassis(12, 12)
motor left_chassis1 = motor(PORT1, ratio6_1, true);
motor left_chassis2 = motor(PORT2, ratio6_1, true);      // dummy
motor left_chassis3 = motor(PORT14, ratio6_1, false);    // dummy
motor_group left_chassis = motor_group(left_chassis1, left_chassis2, left_chassis3);
motor right_chassis1 = motor(PORT10, ratio6_1, false);
motor right_chassis2 = motor(PORT15, ratio6_1, false);   // dummy
motor right_chassis3 = motor(PORT16, ratio6_1, true);    // dummy
motor_group right_chassis = motor_group(right_chassis1, right_chassis2, right_chassis3);

vex::aivision aiCam(vex::PORT5);
// PORT6 = Radio (managed by VEX system, no constructor needed)
inertial inertial_sensor = inertial(PORT12);

digital_out example_piston = digital_out(Brain.ThreeWirePort.A);

// Format is rotation(port, reversed)
// just set these to random ports if you don't use tracking wheels
rotation horizontal_tracker = rotation(PORT18, true);    // dummy
rotation vertical_tracker = rotation(PORT19, true);      // dummy

// Distance reset sensors
// Set these to random ports if you are not using distance resets
distance front_sensor = distance(PORT17);                // dummy
distance left_sensor  = distance(PORT20);                // dummy
distance right_sensor = distance(PORT21);                // dummy
distance back_sensor  = distance(PORT9);                 // dummy

// Clawbot motors
motor arm_motor1 = motor(PORT8, ratio18_1, false);       // arm motor - clawbot standard port
motor_group arm_motor = motor_group(arm_motor1);
motor claw_motor = motor(PORT3, ratio18_1, false);       // claw motor - clawbot standard port

// Unused template devices (kept for compatibility, dummy ports)
digital_out claw = digital_out(Brain.ThreeWirePort.B);
digital_out rush_arm = digital_out(Brain.ThreeWirePort.C);
digital_out mogo_mech = digital_out(Brain.ThreeWirePort.D);
motor intake_motor = motor(PORT4, ratio18_1, true);      // dummy
optical optical_sensor = optical(PORT7);                 // dummy
distance intake_distance = distance(PORT11);             // dummy
distance clamp_distance = distance(PORT13);              // dummy

// ============================================================================
// USER-CONFIGURABLE PARAMETERS (CHANGE BEFORE USING THIS TEMPLATE)
// ============================================================================

// Distance between the middles of the left and right wheels of the drive (in inches)
double distance_between_wheels = 12.3;

// motor to wheel gear ratio * wheel diameter (in inches) * pi
double wheel_distance_in = (36.0 / 48.0) * 3.17 * M_PI;

// PID Constants for movement
// distance_* : Linear PID for straight driving
// turn_*     : PID for turning in place
// heading_correction_* : PID for heading correction during linear movement
double distance_kp = 1.1, distance_ki = 0.1, distance_kd = 7;
double turn_kp = 0.3, turn_ki = 0, turn_kd = 2.5;
double heading_correction_kp = 0.6, heading_correction_ki = 0, heading_correction_kd = 4;

// Enable or disable the use of tracking wheels
bool using_horizontal_tracker = false;  // Set to true if a horizontal tracking wheel is installed and used for odometry
bool using_vertical_tracker = false;    // Set to true if a vertical tracking wheel is installed and used for odometry

// IGNORE THESE IF YOU ARE NOT USING TRACKING WHEELS
double horizontal_tracker_dist_from_center = 2.71875;
double vertical_tracker_dist_from_center = -0.03125;
double horizontal_tracker_diameter = 1.975;
double vertical_tracker_diameter = 1.975;

// Distance Reset setup
double front_sensor_offsetX = 0.0; 
double front_sensor_offsetY = 0.0;
double left_sensor_offsetX = 0.0; 
double left_sensor_offsetY = 0.0;
double right_sensor_offsetX = 0.0;
double right_sensor_offsetY = 0.0;
double back_sensor_offsetX = 0.0;
double back_sensor_offsetY = 0.0;

// ============================================================================
// ADVANCED TUNING (OPTIONAL)
// ============================================================================

bool heading_correction = true;
bool dir_change_start = true;
bool dir_change_end = true;
double min_output = 10;
double max_slew_accel_fwd = 24;
double max_slew_decel_fwd = 24;
double max_slew_accel_rev = 24;
double max_slew_decel_rev = 24;
double chase_power = 2;

// ============================================================================
// DO NOT CHANGE ANYTHING BELOW
// ============================================================================

bool RemoteControlCodeEnabled = true;

void vexcodeInit(void) {
  // nothing to initialize
}
