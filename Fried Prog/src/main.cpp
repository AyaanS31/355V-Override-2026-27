#include "main.h"
#include "rw-config.h"

// RW-Template: your code goes in src/rw-user.cpp (pre-auton, auton, driver),
// src/rw-autons.cpp (routines) and src/rw-config.cpp (ports and tuning).

/**
 * Runs as soon as the program starts: calibrates the IMU, starts odometry
 * and shows the auton selector on the Brain screen.
 */
void initialize() {
	runPreAutonomous();
}

/**
 * If the timer cut the auton off, let the robot coast to a stop, then show where it ended.
 */
void disabled() {
	if (auton_in_progress) {
		pros::delay(500);
		showFinalPose();
		stopChassis(pros::MotorBrake::coast);  // Release the hold from the auton's last move
	}
}

void competition_initialize() {}

/**
 * Runs the routine picked on the auton selector.
 */
void autonomous() {
	runAutonomous();
	showLivePoseUntilAutonEnds();  // Wheels are free now: push the robot and read its coordinates
}

/**
 * Driver control.
 */
void opcontrol() {
	runDriver();
}