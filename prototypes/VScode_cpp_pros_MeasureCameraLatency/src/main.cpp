
#include "main.h"

// Don't worry if this line shows a yellow squiggly.
// If it compiles OK this include file was found OK, but for some reason
// Intellisense thinks it isn't used anywhere. If you delete the include as
// mentioned in the "Quick Fix" you will soon find that it is used quite a lot
#include "lemlib/api.hpp"

#include "pros/abstract_motor.hpp"
#include <string>

// Note:  If you see a lot of blue squigglys in the code below, they indicate
//        failure to pass spell check
//        I personally don't like this feature, and if you agree you can go to
//        File --> Preferences and uncheck "C++: Spell Check: Enabled" to turn
//        it off

// ==========================================
// 1. HARDWARE SETTINGS (Change ports to match your bot)
// ==========================================
// Define the physical motors. Format: (Port, Gearset, Reversed)
pros::Motor
    left_motor(-1,
               pros::v5::MotorGears::ratio_18_to_1); // Left motor on port 1
                                                     // (minus means reversed)
pros::Motor
    right_motor(10,
                pros::v5::MotorGears::ratio_18_to_1); // Right motor on port 10

// Wrap them into LemLib-friendly Motor Groups
pros::MotorGroup left_motors({left_motor});
pros::MotorGroup right_motors({right_motor});

// Define the Inertial/Gyro Sensor
pros::Imu inertial_sensor(12); // Gyro plugged into Smart Port 12

// ==========================================
// 2. LEMLIB DRIVETRAIN PROFILE
// ==========================================
lemlib::Drivetrain drivetrain{
    &left_motors,  // Pointer to left motor group
    &right_motors, // Pointer to right motor group
    10.5, // Track width: Distance between left & right wheels (in inches)
    lemlib::Omniwheel::NEW_4, // Wheel diameter: Clawbots typically use standard
                              // 4" wheels
    200, // Drivetrain RPM: Standard Green V5 cartridges run at 200 RPM
    0.5  // Voltage limit (0.0 to 1.0, where 1.0 corresponds to 12V)
};

// ==========================================
// 3. PID TUNING CONTROLLERS
// ==========================================
// Controls forward/backward movements
lemlib::ControllerSettings lateral_controller(
    10, 0, 3, 3, // kP, kI, kD, windup
    1, 100,      // Small error range (1 inch), small error timeout (100ms)
    3, 500,      // Large error range (3 inches), large error timeout (500ms)
    20           // Slew rate (prevents sudden aggressive acceleration slips)
);

// Controls turning/gyro movements
lemlib::ControllerSettings angular_controller(
    2, 0, 10, 3, // kP, kI, kD, windup
    1, 100,      // Small error range (1 degree), small error timeout (100ms)
    3, 500,      // Large error range (3 degrees), large error timeout (500ms)
    20           // Slew rate
);

// ==========================================
// 4. ODOMETRY SENSORS & CHASSIS CREATION
// ==========================================
lemlib::OdomSensors sensors{
    nullptr,         // No vertical tracking wheel
    nullptr,         // No horizontal tracking wheel
    nullptr,         // No dedicated left encoder
    nullptr,         // No dedicated right encoder
    &inertial_sensor // Pass the gyro pointer for accurate rotational data
};

pros::v5::AIVision myAIVision5(5);

// Create the unified LemLib Chassis object
lemlib::Chassis chassis(drivetrain, lateral_controller, angular_controller,
                        sensors);

pros::Controller master(pros::E_CONTROLLER_MASTER);

FILE *logFile = nullptr;           // Global file pointer for logging
std::vector<std::string> testData; // Vector to store test data for later saving
bool isReadyToTest = false;
double initialHeadingToTag = 0.0;
int prevLapCounter = 0;
const double INVALID_HEADING = -999.0;

/*******************************************************/
// Test settings

const int tagIDToTrack = 1; // Specify only one tag so data is more consistent

const int testSpeed =
    60; // Base speed for the test (0-127, where 127 is full voltage)

/*******************************************************/
// Local functions

void writeLog(std::string data) {
  testData.push_back(data); // Store the data in the vector for later saving
}

void dumpLogToTerminal() {
  for (const std::string &s : testData) {
    std::cout << s << "\n";
  }
}

std::string generateLogFileName() {
  std::time_t now = std::time(nullptr);
  std::tm *t = std::localtime(&now);

  std::ostringstream oss;
  oss << "/usd/" << std::put_time(t, "%Y%m%d_%H%M") << ".log";

  return oss.str();
}

void resetLog() {
  // Open the file in write mode
  FILE *logFile = fopen(generateLogFileName().c_str(), "w");

  if (logFile == NULL) {
    printf("Failed to open log file for writing.\n");
    return;
  }

  // Write each string in the vector as a new line in the CSV
  for (const std::string &line : testData) {
    fprintf(logFile, "%s\n", line.c_str());
  }

  fclose(logFile);

  testData.clear(); // Clear the in-memory log data
}

bool isImuValid() {
    // Check if still calibrating
    if (inertial_sensor.is_calibrating()) {
        printf("IMU still calibrating\n");
        return false;
    }

    // Check heading is a real number (returns PROS_ERR_F if disconnected)
    double heading = inertial_sensor.get_heading();
    if (heading == PROS_ERR_F) {
        printf("IMU returning invalid data\n");
        return false;
    }

    return true;
}

// Compact timestamp in text form to include in log output
std::string getTimestamp() {
  std::time_t now = std::time(nullptr);
  std::tm *t = std::localtime(&now);

  // Round minutes down to nearest 20
  int roundedMin = (t->tm_min / 20) * 20;

  std::ostringstream oss;
  oss << std::put_time(t, "%H")                          // hour
      << std::setw(2) << std::setfill('0') << roundedMin // rounded minutes
      << std::put_time(t, "%S");                         // seconds

  return oss.str(); // e.g. "153045" = 15:30:45
}

double getTagHeading(int id) {
  double tagHeading = INVALID_HEADING;
  std::vector<pros::AIVision::Object> objects = myAIVision5.get_all_objects();

  for (auto &obj : objects) {
    if ((pros::AIVision::is_type(obj, pros::AivisionDetectType::tag)) &&
        (obj.id == id)) {

      // Calculate angle from top edge of tag (corners 0->1)
      double dx = obj.object.tag.x1 - obj.object.tag.x0;
      double dy = obj.object.tag.y1 - obj.object.tag.y0;
      tagHeading = std::atan2(dy, dx) * (180.0 / M_PI);

      // Normalize to 0-359
      if (tagHeading < 0) {
        tagHeading += 360.0;
      }
      break;
    }
  }
  return tagHeading;
}

std::string prepareForTest() {
  isReadyToTest = false;
  std::string errRet = "READY TO TEST";

  double tagHeading = getTagHeading(tagIDToTrack);
  if (tagHeading == INVALID_HEADING) {
    errRet = "Tag not found. Make sure tag is visible to camera. ";
  } else {
    if (!isImuValid()) {
      errRet = "IMU not valid. Check connections and calibration.";
    } else {
      double gyroHeading = inertial_sensor.get_rotation();
      initialHeadingToTag = gyroHeading - tagHeading; // Calculate the
                                                      // initial heading
                                                      // to the tag
      isReadyToTest = true;
    }
  }

  if (!isReadyToTest) {
    errRet = "IMU not ready. Check connections and calibration.";
  }

  return errRet;
}

void runTest() {

  left_motors.move(-1 * testSpeed);
  right_motors.move(testSpeed);

  pros::imu_gyro_s_t gyro = inertial_sensor.get_gyro_rate();
  double currentAngularVelocity = gyro.z;

  double currentHeading = inertial_sensor.get_rotation();
  double tag_heading = getTagHeading(tagIDToTrack);
  double gyroHeadingToTag = currentHeading - initialHeadingToTag;

  writeLog(getTimestamp() + "," + std::to_string(currentAngularVelocity) + "," +
           std::to_string(gyroHeadingToTag) + "," +
           std::to_string(tag_heading) + "," + std::to_string(currentHeading) +
           "\n");
}

/*******************************************************/
// VEXcode generated functions

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();

  // Initialize the gyro sensor
  int initCount = 500; // Time out after 10 seconds
  while (inertial_sensor.is_calibrating()) {
    pros::delay(20);
    initCount--;
    if (initCount <= 0) {
      printf("IMU calibration timed out\n");
      break;
    }
  }
  pros::delay(100); // small extra settle time after calibration reports done

  // Initialize the chassis (important for odometry to work correctly)
  chassis.calibrate();

  myAIVision5.reset();
  pros::delay(100);

  myAIVision5.enable_detection_types(pros::AivisionModeType::tags);
  myAIVision5.set_tag_family(pros::AivisionTagFamily::tag_61H11);

  // Clear any existing log data
  testData.clear();
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous
 * selector on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own
 * task with the default priority and stack size whenever the robot is enabled
 * via the Field Management System or the VEX Competition Switch in the
 * autonomous mode. Alternatively, this function may be called in initialize
 * or opcontrol for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start
 * it from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own
 * task with the default priority and stack size whenever the robot is enabled
 * via the Field Management System or the VEX Competition Switch in the
 * operator control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart
 * the task, not resume it from where it left off.
 */
void opcontrol() {
  while (true) {

    if (master.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_X)) {

      // Button X was just released
      dumpLogToTerminal(); // Send log data to the terminal window
      // (only works if connected by USB)

    } else if (master.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_Y)) {

      // Button Y was just released
      resetLog(); // Clear the log data after saving to SD card

    } else if (master.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_A)) {

      // Button A was just released — Prepare for test
      std::string errMsg = prepareForTest();
      writeLog(errMsg + "\n");
      pros::lcd::set_text(2, errMsg);

    } else if (master.get_digital_new_release(pros::E_CONTROLLER_DIGITAL_B)) {

      // Button B was just released - End of test
      left_motors.move(0);  // Stop
      right_motors.move(0); // Stop

      writeLog("TEST END\n");
      pros::lcd::set_text(2, "TEST ENDED");

    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {

      // Holding B button down - run the test
      runTest();

    } else {
      // Arcade control scheme
      int dir = master.get_analog(ANALOG_RIGHT_Y);  // forward/backward
      int turn = master.get_analog(ANALOG_RIGHT_X); // left/right

      left_motors.move(dir - turn);  // Sets left motor voltage
      right_motors.move(dir + turn); // Sets right motor voltage

      if (turn < -5) { // Push stick to the left for testing motion blur

        pros::imu_gyro_s_t gyro = inertial_sensor.get_gyro_rate();
        double currentAngularVelocity = gyro.z;
        double tag_heading = getTagHeading(tagIDToTrack);

        writeLog(getTimestamp() + ",TURN," + "," +
                 std::to_string(currentAngularVelocity) + "," +
                 std::to_string(tag_heading) + "\n");
      }
    }

    pros::delay(20); // Run for 20 ms then update
  }
}
