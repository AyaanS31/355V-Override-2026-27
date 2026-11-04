#include "rw-config.h"
#include "rw/hardware.h"

// Modify autonomous, driver, or pre-auton code below. src/main.cpp calls these.

extern bool selector_running;  // RW's auton selector (rw/auton-selector.cpp)

// True once an auton has run, so runDriver() keeps the position it ended at
static bool auton_has_run = false;

void runAutonomous() {
  // Runs the routine picked on the Brain screen (or with the controller arrows while disabled).
  // Add routines to the list in runPreAutonomous() below.
  auton_in_progress = true;
  auton_has_run = true;
  runSelectedAuton();
  showFinalPose();
  // Motions end in hold mode, which would keep the wheels locked until the auton period ends
  stopChassis(pros::MotorBrake::coast);
}

// Controller sticks read -127 to 127; RW's driver functions take -100 to 100
double stick(pros::controller_analog_e_t axis) {
  return controller_1.get_analog(axis) * 100.0 / 127.0;
}

void runDriver() {
  // In case the auton was cut off before it could stop the pose display (it would fight the prints below)
  stopPoseDisplay();
  showFinalPose();
  if (!auton_has_run) setSkillsStartPose();
  stopChassis(pros::MotorBrake::coast);
  heading_correction = false;
  bool selector_waiting = false;
  while (true) {
    // default tank drive or replace it with your preferred driver code here:
    driverTank(stick(pros::E_CONTROLLER_ANALOG_LEFT_Y), stick(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
    // Arcade on the left stick:           driverArcade(stick(pros::E_CONTROLLER_ANALOG_LEFT_Y), stick(pros::E_CONTROLLER_ANALOG_LEFT_X));
    // Split arcade (left fwd, right turn): driverArcade(stick(pros::E_CONTROLLER_ANALOG_LEFT_Y), stick(pros::E_CONTROLLER_ANALOG_RIGHT_X));
    // X-drive, mecanum or H-drive:         driverHolonomic(stick(pros::E_CONTROLLER_ANALOG_LEFT_Y), stick(pros::E_CONTROLLER_ANALOG_LEFT_X),
    //                                                     stick(pros::E_CONTROLLER_ANALOG_RIGHT_X));  (or driverFieldCentric)
    // Tune feel with driver_curve and driver_deadband in src/rw-config.cpp
    // Lift: L1 up, L2 down, holds in place when released
    if (controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
      lift_right.move(127);   // Lift goes up
      lift_right2.move(127);
      lift_left.move(127);
    } else if (controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
      lift_right.move(-127);  // Lift goes down
      lift_right2.move(-127);
      lift_left.move(-127);
    } else {
      lift_right.brake();     // Lock the lift in place (hold mode, set in runPreAutonomous)
      lift_right2.brake();
      lift_left.brake();
    }


    // Clamp: Y toggles it
    if (controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      clamp.toggle();
    }

    // Toggle piston: Right arrow flips it down / back up
    if (controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
      toggle.toggle();
    }

    // Practice only: X runs the selected auton when no competition switch or field is plugged in.
    // Afterwards the final position stays on the Brain screen. Press A to bring the selector back
    // and tap another routine
    if (!pros::competition::is_connected() && controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      heading_correction = true;
      runAutonomous();
      stopChassis(pros::MotorBrake::coast);
      heading_correction = false;
      selector_waiting = true;
    }
    if (selector_waiting && controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      selector_waiting = false;
      startAutonSelector();
    }

    // Down toggles the live sensor view on the Brain screen. It starts with the auto reset off, so the
    // position is pure odometry and you can see what each sensor says; Up turns the auto reset on/off
    static bool sensor_view = false;
    static double last_sensor_draw = 0;
    // Watches the button itself instead of get_digital_new_press(), so a press can't get missed
    static bool down_was_pressed = false;
    bool down_pressed = controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
    bool down_new_press = down_pressed && !down_was_pressed;
    down_was_pressed = down_pressed;
    if (down_new_press) {
      sensor_view = !sensor_view;
      auto_reset_enabled = false;
      if (sensor_view) selector_running = false;  // Stop the selector drawing over the view
      hw::screenClear(0x000000);
    }
    if (sensor_view && controller_1.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      auto_reset_enabled = !auto_reset_enabled;
    }
    if (sensor_view && pros::millis() - last_sensor_draw > 100) {
      last_sensor_draw = pros::millis();
      drawDistanceSensors();
      hw::screenRow(9, "Down = close view   Up = auto reset on/off");
    }

    // Controller screen: robot position on the top line, skills start position below it, sensor view
    // status on the bottom. The controller only accepts a print every 50 msec, so the lines take turns
    static double last_print = 0;
    static int print_line = 0;
    if (pros::millis() - last_print > 60) {
      last_print = pros::millis();
      if (print_line == 0) {
        printPoseLine();
      } else if (print_line == 1) {
        printSkillsStartLine();
      } else if (sensor_view) {
        controller_1.print(2, 0, "SENSORS ON rst %-3s  ", auto_reset_enabled ? "on" : "off");
      } else {
        controller_1.print(2, 0, "Down = sensor view ");
      }
      print_line = (print_line + 1) % 3;
    }

    pros::delay(10);
  }
}

void runPreAutonomous() {
  // Calibrate the inertial sensor and wait for it (about 2 seconds)
  inertial_sensor.reset(true);

  // Lift holds its position when stopped
  lift_right.set_brake_mode(pros::MotorBrake::hold);
  lift_right2.set_brake_mode(pros::MotorBrake::hold);
  lift_left.set_brake_mode(pros::MotorBrake::hold);

  // X-drive or mecanum: useHolonomicDrive(left_front, right_front, left_back, right_back);
  // H-drive: useHDrive(strafe_motor);   (see the docs: Guide > Drive Types)

  // odom tracking (picks the setup from using_horizontal_tracker / using_vertical_tracker)
  startOdometry();

  // Distance sensor auto reset (src/rw-distance.cpp): keeps x and y lined up with the walls.
  // Use this or startMcl() below, not both
  // startAutoReset();  // Off: positions are kept with setPose() instead of distance sensors

  // Optional: MCL uses the distance sensors to keep odometry from drifting while driving.
  // Needs field-centered coordinates, so start each auton with setPose() at the real field position.
  // startMcl();

  // Autonomous routines for the selector, in the order they appear on the Brain screen
  // addAuton("Left Side", leftSide);
  // addAuton("Right Side", rightSide);
  addAuton("Skills", skills);
  addAuton("Pose Check", poseCheck);
  addAuton("Distance Check", distanceCheck);
  addAuton("IMU Scale", imuScaleTest);
  addAuton("Turn Tune", turnTuningTest);
  addAuton("Drive Tune", driveTuningTest);
  addAuton("Drive Calibration", calibrateDrive);  // For the simulator (see the docs: Simulator > Calibrating)
  startAutonSelector();
}