#include "rw-config.h"
#include "rw/hardware.h"
#include <algorithm>
#include <cmath>

// IMPORTANT: Remember to add respective function declarations to include/rw-config.h
// Then add each routine to the auton selector with addAuton() in runPreAutonomous() (src/rw-user.cpp)
// Format: returnType functionName() { code }

// Where myAuton starts on the field (Pose Check starts here too, so its x and y match the auton's)
const double my_start_x = -1.035, my_start_y = -63.953, my_start_heading = 0;

void leftSide(){
    // check Pose and see if it works. try auton on field.
  setPose(my_start_x, my_start_y, my_start_heading);
  lift_right.move(127);
  lift_left.move(127);
  pros::delay(330);
  lift_right.brake();
  lift_left.brake();
  moveToPoint(-1, -52, 1, 2000, true);
  moveToPoint(-1, -72, -1, 1200, true);
  moveToPoint(-1, -52, 1, 1800, true);
  moveToPoint(-1, -72, -1, 1200, true);
  moveToPoint(-1, -52, 1, 2000, true);
  turnToAngle(-90, 1000);
  pros::delay(500);
  moveToPoint(-23, -51, 1 , 2000, true, 5);
  lift_left.move(-127);
  lift_right.move(-127);
  pros::delay(350);
  lift_left.brake();
  lift_right.brake();
  clamp.toggle();
  pros::delay(350);
  moveToPoint(-21, -51, -1 , 2000, true, 5);
}

// leftSide mirrored to the right side of the field: every x flips sign and the turn goes the other way


// Controller pose display (see startPoseDisplay)
bool pose_display_on = false;
bool pose_display_running = false;

// Prints x, y and heading (0-360) on the top line of the controller screen.
// Used by the auton display below and by runDriver(), so the position is always on the same line
void printPoseLine() {
  double heading = std::fmod(getInertialHeading(), 360.0);
  if (heading < 0) heading += 360;
  controller_1.print(0, 0, "x%5.1f y%5.1f h%3.0f ", x_pos, y_pos, heading);
}

// Shows the position on the controller screen in the background until stopPoseDisplay().
// runDriver() is paused while an auton runs, so without this the screen would freeze.
// The controller only accepts a print every 50 msec, so the lines take turns
void startPoseDisplay() {
  pose_display_on = true;
  if (pose_display_running) return;
  pose_display_running = true;
  pros::Task([] {
    while (pose_display_on) {
      printPoseLine();
      pros::delay(60);
      controller_1.print(1, 0, "auton running      ");
      pros::delay(60);
      controller_1.print(2, 0, "auto resets %4d   ", auto_reset_count);
      pros::delay(60);
    }
    pose_display_running = false;
  });
}

void stopPoseDisplay() {
  pose_display_on = false;
}

// True from when an auton starts until showFinalPose() has shown where it ended
bool auton_in_progress = false;

// Where the robot was each time it clamped at the loader (before the heading reset), shown under the
// final position so you can see if it lines up the same every cycle
static const int max_loader_log = 5;
static double loader_log[max_loader_log][3];
static int loader_visits = 0;

// Shows where the auton ended on the Brain screen. Called when the routine finishes, and from
// disabled() and runDriver() in case the timer cut it off, so it only draws once per run
void showFinalPose() {
  if (!auton_in_progress) return;
  auton_in_progress = false;
  stopPoseDisplay();
  double heading = std::fmod(getInertialHeading(), 360.0);
  if (heading < 0) heading += 360;
  hw::screenClear(0x000000);
  hw::screenRow(1, "AUTON ENDED - FINAL POSITION");
  hw::screenRow(3, "X: %.1f in", x_pos);
  hw::screenRow(4, "Y: %.1f in", y_pos);
  hw::screenRow(5, "Heading: %.1f deg", heading);
  for (int i = 0; i < loader_visits && i < max_loader_log; i++) {
    hw::screenRow(7 + i, "Loader %d:  x %.1f  y %.1f  h %.1f", i + 1, loader_log[i][0], loader_log[i][1],
                  loader_log[i][2]);
  }
}

// Keeps showing the position after the auton ends, so you can push the robot to a spot and read its
// coordinates. Never returns: PROS ends it when the autonomous period does (or the switch is flipped)
void showLivePoseUntilAutonEnds() {
  while (true) {
    printPoseLine();
    double heading = std::fmod(getInertialHeading(), 360.0);
    if (heading < 0) heading += 360;
    hw::screenRow(6, "NOW  X: %.1f  Y: %.1f  H: %.1f   ", x_pos, y_pos, heading);
    pros::delay(100);
  }
}

// PATH.JERRYIO Override Skills field coordinates, mirrored so the red goals are on +X.
// Keep the robot center about 7.5 in from the wall; x=-65 would put a 15 in robot outside the field.
static const double skills_start_x = -62.0;
static const double skills_start_y = 0.0;
static const double skills_start_heading = 90.0;

// Starts the position readout at the skills start, so driving around shows real field coordinates
void setSkillsStartPose() {
  setPose(skills_start_x, skills_start_y, skills_start_heading);
}

// Prints the skills start position on the second line of the controller screen
void printSkillsStartLine() {
  controller_1.print(1, 0, "start x%5.1f y%5.1f ", skills_start_x, skills_start_y);
}

// Red-side goals and the yellow pins on the diagonal beside them (tile pitch = 23.545 in).
static const double red_goal_near_x = 47.09, red_goal_near_y = -23.545;
static const double red_goal_far_x = 23.545, red_goal_far_y = -47.09;
static const double yellow_pin_near_x = 23.545, yellow_pin_near_y = -23.545;
static const double yellow_pin_far_x = 47.09, yellow_pin_far_y = -47.09;

// The clamp grabs each pin at the lowered lift position; the lift raises it to score.
static void lowerSkillsLift() {
  lift_right.move(-127);
  lift_right2.move(-127);
  lift_left.move(-127);
  pros::delay(350);
  lift_right.brake();
  lift_right2.brake();
  lift_left.brake();
}

static void raiseSkillsLift() {
  lift_right.move(127);
  lift_right2.move(127);
  lift_left.move(127);
  pros::delay(330);
  lift_right.brake();
  lift_right2.brake();
  lift_left.brake();
}

static void collectSkillsPin() {
  lowerSkillsLift();
  clamp.toggle();
  pros::delay(150);
}

static void scoreSkillsPin() {
  raiseSkillsLift();
  pros::delay(150);
  clamp.toggle();
  pros::delay(150);
  lowerSkillsLift();
}

// Stops at a checkpoint and lets the auto reset correct x and y from the walls.
// Does nothing but wait until the sensor offsets are set in src/rw-config.cpp
static void wallReset() {
  pros::delay(100);  // Let the robot settle
  auto_reset_enabled = true;
  pros::delay(400);  // 5 readings must agree, then it corrects every 30 msec
  auto_reset_enabled = false;
}

// Where the robot stops when it drives into the loader: put the x from the controller during a good
// run here. The loader stops the robot in the same place every time, so x goes back to this value
// each cycle and the drift is cleared. 0 = not set yet, so loaderSetPose() does nothing
static const double loader_x = -67.2;

// Call right after driving into the loader, before clamping. Keeps y and heading as they are
static void loaderSetPose() {
  if (loader_x == 0) return;
  setPose(loader_x, y_pos, getInertialHeading());
}

// Enters the loader gently instead of ramming it: drives to a few inches short, then creeps in at
// low power for a fixed time so the robot settles against the loader the same way every cycle.
// Raise loader_creep_volts if it doesn't reach the loader, lower it if the robot still shoves.
// y: the same y the robot backed up to before the turn, so it drives straight in instead of at an angle
static const double loader_creep_volts = 2;
static void enterLoader(double y) {
  moveToPoint(-63, y, 1, 1700, true, 4.5);  // Stop about 4 inches short
  driveChassis(loader_creep_volts, loader_creep_volts);
  // Creep for up to 700 msec, but stop once the robot has stopped against the loader
  const double creep_start = pros::millis();
  while (pros::millis() - creep_start < 700) {
    if (pros::millis() - creep_start > 300 && odom_speed_ips < 1) break;
    pros::delay(10);
  }
  stopChassis(pros::MotorBrake::hold);
  pros::delay(100);  // Let it settle before resetting and clamping
  // Log where odometry thinks it is before the x reset, for the Brain screen after the auton
  if (loader_visits < max_loader_log) {
    loader_log[loader_visits][0] = x_pos;
    loader_log[loader_visits][1] = y_pos;
    loader_log[loader_visits][2] = getInertialHeading();
  }
  loader_visits++;
  loaderSetPose();
}

// turnToAngle() counts a turn as done once it's within 3 degrees for 250 msec, so turns can end
// 1-3 degrees short. This turns, then nudges the robot at low power until it's within 0.75 degrees
// (at most 300 msec more). Same arguments as turnToAngle()
static void turnPrecise(double angle, double time_ms, bool exit, double volts) {
  turnToAngle(angle, time_ms, exit, volts);
  const double start = pros::millis();
  while (pros::millis() - start < 300) {
    double error = std::remainder(angle - getInertialHeading(), 360.0);
    if (std::fabs(error) < 0.75) break;
    // Enough power to get the robot moving, a little more the further off it is
    double power = std::copysign(std::fmin(2 + 0.5 * std::fabs(error), 4), error);
    driveChassis(power, -power);
    pros::delay(10);
  }
  stopChassis(pros::MotorBrake::hold);
}

// The loader checkpoint: after backing up to (-53, -59) and turning to face the loader, the front
// sensor sees the loader's face and the left sensor sees the side wall. These are the readings from a
// good run at that spot, and where odometry said the robot was. Each later visit compares its readings
// with these, so x and y are corrected without needing sensor offsets or real field coordinates
static const double checkpoint_x = -52.63, checkpoint_front = 17.8;
static const double checkpoint_y = -60.14, checkpoint_left = 5.2;

// Takes the middle of 5 readings in inches. Returns false if any reading saw nothing
// or they disagree by more than half an inch
static bool steadyReading(double readings[5], double& reading_in) {
  for (int i = 0; i < 5; i++) {
    if (readings[i] <= 0 || readings[i] >= 9999 / 25.4) return false;
  }
  std::sort(readings, readings + 5);
  reading_in = readings[2];
  return readings[4] - readings[0] < 0.5;
}

// Call at the loader checkpoint, stopped and facing the loader. Takes about 0.25 sec: both sensors
// are read at the same time. A reading more than 4 inches from what's expected is skipped
// (probably seeing something else)
static void checkpointReset() {
  pros::delay(50);  // Let the robot settle
  double front_readings[5], left_readings[5];
  for (int i = 0; i < 5; i++) {
    front_readings[i] = front_sensor.get() / 25.4;
    left_readings[i] = left_sensor.get() / 25.4;
    pros::delay(35);
  }
  double x = x_pos, y = y_pos, reading = 0;
  // Facing the loader: a bigger front reading means further from it (larger x)
  if (steadyReading(front_readings, reading)) {
    double new_x = checkpoint_x + (reading - checkpoint_front);
    if (std::fabs(new_x - x_pos) < 4) x = new_x;
  }
  // The left sensor faces the side wall: a bigger reading means further from it (larger y)
  if (steadyReading(left_readings, reading)) {
    double new_y = checkpoint_y + (reading - checkpoint_left);
    if (std::fabs(new_y - y_pos) < 4) y = new_y;
  }
  printf("checkpoint reset: x %.2f -> %.2f, y %.2f -> %.2f\n", x_pos, x, y_pos, y);
  setPose(x, y, getInertialHeading());
}

// Drives the lift back to where it was when skills() started, using the motor encoders.
// Gives up after 1500 msec so a stuck lift can't hold up the auton or overheat the motors
static void lowerLiftFully() {
  lift_right.move_absolute(0, 200);
  lift_right2.move_absolute(0, 200);
  lift_left.move_absolute(0, 200);
  const double start = pros::millis();
  while (pros::millis() - start < 1500 &&
         (std::fabs(lift_right.get_position()) > 10 || std::fabs(lift_right2.get_position()) > 10 ||
          std::fabs(lift_left.get_position()) > 10)) {
    pros::delay(10);
  }
  lift_right.brake();
  lift_right2.brake();
  lift_left.brake();
}

// Lift moves that run in the background, so the lift moves while the robot turns or drives instead
// of the robot sitting still. Call waitForLift() before anything that needs the lift finished
static volatile bool lift_busy = false;

// Runs the lift at a power for a set time, same as doing it in place, so it ends at the same height.
// start_delay_ms waits a little first (e.g. until a turn has swung the robot away from the loader)
static void liftForAsync(int power, int ms, int start_delay_ms = 0) {
  lift_busy = true;
  pros::Task([power, ms, start_delay_ms] {
    pros::delay(start_delay_ms);
    lift_right.move(power);
    lift_right2.move(power);
    lift_left.move(power);
    pros::delay(ms);
    lift_right.brake();
    lift_right2.brake();
    lift_left.brake();
    lift_busy = false;
  });
}

static void lowerLiftFullyAsync() {
  lift_busy = true;
  pros::Task([] {
    lowerLiftFully();
    lift_busy = false;
  });
}

static void waitForLift() {
  while (lift_busy) pros::delay(10);
}

// Drives from the left-wall toggle to the two mirrored red goals.
// The path clears the center goal and approaches each goal from an open side.
void skills() {
  startPoseDisplay();
  const bool previous_auto_reset = auto_reset_enabled;
  auto_reset_enabled = false;  // Off while driving; wallReset() turns it on at checkpoints
  setPose(skills_start_x, skills_start_y, skills_start_heading);
  loader_visits = 0;  // So the first loader visit this run sets the heading
  // Start with the lift all the way down: this spot becomes position 0 for lowerLiftFully()
  lift_right.tare_position();
  lift_right2.tare_position();
  lift_left.tare_position();

// score first pin
  clamp.toggle();
  toggle.toggle();
  moveToPoint(-48, 0, 1, 2050, true, 7);
  lift_right.move(127);
  lift_right2.move(127);
  lift_left.move(127);
  pros::delay(430);
  lift_right.brake();
  lift_right2.brake();
  lift_left.brake();
  // The wall stops the robot before it reaches x -63, so end each toggle push once the
  // robot has been stopped for 250 msec instead of waiting out the whole timeout
  stall_exit_msec = 250;
  moveToPoint(-63.3, 0, -1, 1300, true, 5);
  pros::delay(200);
  moveToPoint(-48, 0, 1, 2050, true, 7);
  moveToPoint(-63.5, 0, -1, 1300, true, 4.5);
  stall_exit_msec = 0;
  moveToPoint(-50, 0, 1, 2050, true, 7);
  turnPrecise(180, 800, true, 7);
  moveToPoint(-48, -19.6, 1, 1700, true, 7);
  lift_left.move(-127);
  lift_right.move(-127);
  lift_right2.move(-127);
  pros::delay(370);
  lift_left.move(0);
  lift_right.move(0);
  lift_right2.move(0);
  clamp.toggle();
  pros::delay(300);  // Clamp opens before backing away

  // second stack
  moveToPoint(-45, -9, -1, 2050, true, 7);
  turnPrecise(137, 1050, true, 10);
  pros::delay(150);  // turnPrecise already waits until the angle settles
  moveToPoint(-31.3, -21, 1, 1950, true, 7);
  clamp.toggle();
  pros::delay(300);  // Clamp closes before the lift pulls up
  liftForAsync(127, 900);  // Lift goes up during the short move and the turn
  moveToPoint(-30, -23, 1, 2050, true, 6);

  // score second pin
  turnPrecise(270, 1150, true, 5.6);
  moveToPoint(-18, -23, -1, 2050, true, 6);
  moveToPoint(-30, -23, 1, 2050, true, 6);
  waitForLift();
  lift_right.brake();
  lift_right2.brake();
  lift_left.brake();
  moveToPoint(-51.3, -25.4, 1, 1950, true, 7);
  lift_left.move(-70);
  lift_right.move(-70);
  lift_right2.move(-70);
  pros::delay(950);
  lift_right.move(0);
  lift_right2.move(0);
  lift_left.move(0);
  lift_right.brake();
  lift_right2.brake();
  lift_left.brake();
  clamp.toggle();
  pros::delay(300);
  moveToPoint(-31, -22, -1, 2050, true, 7);
  turnPrecise(220, 800, true, 10);
  lift_right.move(0);
  lift_right2.move(0);
  lift_left.move(0);
  moveToPoint(-50.8, -44.2, 1, 2050, true, 4.5);
  clamp.toggle();
  pros::delay(300);  // Clamp closes on the stack
  moveToPoint(-53, -47, 1, 2050, true, 4.5);

  // score third stack
  liftForAsync(100, 950);  // Lift goes up while the robot turns
  turnPrecise(0, 1600, true, 6.5);
  waitForLift();
  moveToPoint(-55.2, -30, 1, 2150, true, 6.5);
  lift_left.move(-90);
  lift_right.move(-90);
  lift_right2.move(-90);
  pros::delay(800);
  lift_right.brake();
  lift_right2.brake();
  lift_left.brake();
  clamp.toggle();
  pros::delay(400);  // Clamp opens before backing away
  // // go to match load

  moveToPoint(-53, -59, -1, 2050, true, 6);
  lowerLiftFullyAsync();  // Lift goes down while the robot turns
  turnPrecise(270, 1200, true, 6.2);
  waitForLift();
  // see where it stops and check based on that.
  checkpointReset();  // Corrects x and y from the loader face and side wall before going in

  enterLoader(-59);
  clamp.toggle();
  pros::delay(350);

  moveToPoint(-53, -59, -1, 2050, true, 6.5);
  liftForAsync(127, 950, 250);  // Lift goes up while turning, once the robot has swung away from the loader
  turnPrecise(0, 1600, true, 6.5);
  waitForLift();
  moveToPoint(-55.2, -30, 1, 2150, true, 6.5);

  lift_right.move(-70);
  lift_right2.move(-70);
  lift_left.move(-70);
  pros::delay(400);
  lift_right.brake();
  lift_right2.brake();
  lift_left.brake();
  clamp.toggle();
  pros::delay(400);  // Pause so the pin settles on the goal before backing out

  // score onto the other goal now (1 pin and 3 stacks)
  moveToPoint(-53, -59, -1, 2050, true, 6.5);
  lowerLiftFullyAsync();  // Make sure the lift is all the way down to take the next stack
  turnPrecise(270, 1050, true, 6.5);
  waitForLift();
  checkpointReset();
  enterLoader(-59);
  clamp.toggle();
  pros::delay(350);


  moveToPoint(-53, -59, -1, 2050, true, 6.5);
  liftForAsync(127, 400, 250);  // Lift goes up while turning, once the robot has swung away from the loader
  turnPrecise(67, 1100, true, 6.5);
  waitForLift();
  // moveToPoint(-34.9, -49.3, 1, 2050, true, 6.5); // go to the matchloading goal
  // lift_right.move(-70);
  // lift_right2.move(-70);
  // lift_left.move(-70);
  // pros::delay(550);
  // lift_right.brake();
  // lift_right2.brake();
  // lift_left.brake();
  // clamp.toggle();
  // pros::delay(400);

  // // scored a pin. 
  // // now score first stack 
  // moveToPoint(-53, -59, -1, 2050, true, 6.5);
  // lowerLiftFullyAsync();  // Lift goes down while the robot turns
  // turnPrecise(271, 1050, true, 6.5);
  // waitForLift();
  // checkpointReset();
  // enterLoader(-59);
  // clamp.toggle();
  // pros::delay(350);  // Clamp closes on the stack
  // moveToPoint(-53, -59, -1, 2050, true, 6.5);
  // liftForAsync(127, 600, 250);  // Lift goes up while turning, once the robot has swung away from the loader
  // turnPrecise(67, 1100, true, 6.5);
  // waitForLift();
  // moveToPoint(-34.9, -49.3, 1, 2050, true, 6.5); // go to the matchloading goal
  // lift_right.move(-70);
  // lift_right2.move(-70);
  // lift_left.move(-70);
  // pros::delay(550);
  // lift_right.brake();
  // lift_right2.brake();
  // lift_left.brake();
  // clamp.toggle();
  // pros::delay(400);

  // // scored first stack
  // // now get second stack and score
  // moveToPoint(-53, -59, -1, 2050, true, 6.5);
  // lowerLiftFullyAsync();  // Lift goes down while the robot turns
  // turnPrecise(271, 1050, true, 6.5);
  // waitForLift();
  // checkpointReset();
  // enterLoader(-59);
  // clamp.toggle();
  // pros::delay(350);  // Clamp closes on the stack
  // moveToPoint(-53, -59, -1, 2050, true, 6.5);
  // liftForAsync(127, 750, 250);  // Lift goes up while turning, once the robot has swung away from the loader
  // turnPrecise(67, 1100, true, 6.5);
  // waitForLift();
  // moveToPoint(-34.9, -49.3, 1, 2050, true, 6.5); // go to the matchloading goal
  // lift_right.move(-70);
  // lift_right2.move(-70);
  // lift_left.move(-70);
  // pros::delay(550);
  // lift_right.brake();
  // lift_right2.brake();
  // lift_left.brake();
  // clamp.toggle();
  // pros::delay(400);

  // // scored second stack
  // // now get third stack and score 
  // moveToPoint(-53, -59, -1, 2050, true, 6.5);
  // lowerLiftFullyAsync();  // Lift goes down while the robot turns
  // turnPrecise(271, 1050, true, 6.5);
  // waitForLift();
  // checkpointReset();
  // enterLoader(-59);
  // clamp.toggle();
  // pros::delay(350);  // Clamp closes on the stack
  // moveToPoint(-53, -59, -1, 2050, true, 6.5);
  // liftForAsync(127, 800, 250);  // Lift goes up while turning, once the robot has swung away from the loader
  // turnPrecise(67, 1100, true, 6.5);
  // waitForLift();
  // moveToPoint(-34.9, -49.3, 1, 2050, true, 6.5); // go to the matchloading goal
  // lift_right.move(-70);
  // lift_right2.move(-70);
  // lift_left.move(-70);
  // pros::delay(550);
  // lift_right.brake();
  // lift_right2.brake();
  // lift_left.brake();
  // clamp.toggle();
  // pros::delay(400);


  // // all stacks for goal done for match load. 
  // // get a pin for center goal and move to toggle point
  // moveToPoint(-53, -59, -1, 2050, true, 6.5);
  // lowerLiftFullyAsync();  // Lift goes down while the robot turns
  // turnPrecise(271, 1050, true, 6.5);
  // waitForLift();
  // checkpointReset();
  // enterLoader(-59);
  // clamp.toggle();
  // pros::delay(350);  // Clamp closes on the stack
  // // go to toggle point 

  // moveToPoint(-9,-58.4, -1, 2050, true, 7);
  // turnPrecise(0, 1050, true, 7);
  // // Same as the first toggle: the wall stops the robot short of y -64.2, so end each push
  // // once it has been stopped for 250 msec instead of waiting out the whole timeout
  // stall_exit_msec = 250;
  // moveToPoint(-9, -64.2, -1, 1300, true, 5.5);
  // moveToPoint(-9, -55.3, 1, 2050, true, 5.5);
  // moveToPoint(-9, -64.2, -1, 1300, true, 6);
  // stall_exit_msec = 0;
  // // Lift goes all the way up while backing off the toggle. Raise center_lift_up_ms if it
  // // doesn't reach the top, lower it if it reaches the top early (saves time)
  // const int center_lift_up_ms = 980;
  // liftForAsync(127, center_lift_up_ms);
  // moveToPoint(-9, -58.4, 1, 2050, true, 5.5);
  // waitForLift();  // At max height before driving to the center goal

  // // // go to center goal and score.
  // moveToPoint(-10.9,-4.0, 1, 2050, true, 7.3);
  // lowerLiftFully();  // All the way down onto the goal (stops the lift motors when done)
  // clamp.toggle();    // Let go of the pin
  // pros::delay(400);

  // stopPoseDisplay();
}

void rightSide(){
  startPoseDisplay();
  const bool previous_auto_reset = auto_reset_enabled;
  auto_reset_enabled = false;  // Off while driving; wallReset() turns it on at checkpoints
  setPose(skills_start_x, skills_start_y, skills_start_heading);
  loader_visits = 0;  // So the first loader visit this run sets the heading
  // Start with the lift all the way down: this spot becomes position 0 for lowerLiftFully()
  lift_right.tare_position();
  lift_right2.tare_position();
  lift_left.tare_position();

// score first pin
  clamp.toggle();
  toggle.toggle();
  moveToPoint(-48, 0, 1, 2050, true, 7);
  lift_right.move(127);
  lift_right2.move(127);
  lift_left.move(127);
  pros::delay(430);
  lift_right.brake();
  lift_right2.brake();
  lift_left.brake();
  // The wall stops the robot before it reaches x -63, so end each toggle push once the
  // robot has been stopped for 250 msec instead of waiting out the whole timeout
  stall_exit_msec = 250;
  moveToPoint(-63.3, 0, -1, 1300, true, 5);
  pros::delay(250);
  moveToPoint(-48, 0, 1, 2050, true, 7);
  moveToPoint(-63.5, 0, -1, 1300, true, 4.5);
  stall_exit_msec = 0;
  moveToPoint(-50, 0, 1, 2050, true, 7);
  turnPrecise(180, 800, true, 10);
  moveToPoint(-50, -20, 1, 1700, true, 7);
  lift_left.move(-127);
  lift_right.move(-127);
  lift_right2.move(-127);
  pros::delay(370);
  lift_left.move(0);
  lift_right.move(0);
  lift_right2.move(0);
  clamp.toggle();
  pros::delay(300);  // Clamp opens before backing away

  // second stack
  moveToPoint(-45, -9, -1, 2050, true, 7);
  turnPrecise(129, 1050, true, 10);
  pros::delay(150);  // turnPrecise already waits until the angle settles
  moveToPoint(-33, -20, 1, 1950, true, 7);
  clamp.toggle();
  pros::delay(300);  // Clamp closes before the lift pulls up
  lift_left.move(127);
  lift_right.move(127);
  lift_right2.move(127);
  pros::delay(900);
  lift_left.brake();
  lift_right.brake();
  lift_right2.brake();
  moveToPoint(-30, -23, 1, 2050, true, 6);

  // score second pin
  turnPrecise(270, 1150, true, 5.6);
  lift_right.brake();
  lift_right2.brake();
  lift_left.brake();
  moveToPoint(-49, -24, 1, 1950, true, 7);
  lift_left.move(-70);
  lift_right.move(-70);
  lift_right2.move(-70);
  pros::delay(950);
  lift_right.move(0);
  lift_right2.move(0);
  lift_left.move(0);
  lift_right.brake();
  lift_right2.brake();
  lift_left.brake();
  clamp.toggle();
  pros::delay(300);
  moveToPoint(-31, -22, -1, 2050, true, 7);
  turnPrecise(220, 800, true, 10);
  lift_right.move(0);
  lift_right2.move(0);
  lift_left.move(0);
  moveToPoint(-49, -43, 1, 2050, true, 4.5);
  clamp.toggle();
  pros::delay(250);
  moveToPoint(-53, -47, 1, 2050, true, 4.5);

  // score third stack
  lift_right.move(127);
  lift_right2.move(127);
  lift_left.move(127);
  pros::delay(900);
  lift_right.brake();
  lift_right2.brake();
  lift_left.brake();
  turnPrecise(0, 1600, true, 4.5);
  moveToPoint(-53, -28, 1, 2150, true, 5);
  lift_left.move(-90);
  lift_right.move(-90);
  lift_right2.move(-90);
  pros::delay(800);
  lift_right.brake();
  lift_right2.brake();
  lift_left.brake();
  clamp.toggle();
  pros::delay(300);  // Let the clamp open before the auton ends
}
/*
  // Drive forward from the left start, then reverse into the left-wall toggle.
 * asyncExample
 * Shows background motions: start a move, run the intake while it drives, and stop it early with a sensor.
 */
void asyncExample() {
  setPose(0, 0, 0);

  // Start driving in the background, then start the intake 12 inches into the move
  runAsync([]{ moveToPoint(0, 48, 1, 3000); });
  waitUntilTraveled(12);


  // Turn around and drive back
  runAsync([]{
    turnToAngle(180, 1500);
    moveToPoint(0, 0, 1, 3000);
  });
  waitForMotion();
}

/*
 * pathExample
 * Drives a smooth S-curve with pure pursuit, then backs along it with RAMSETE.
 * Paths use field coordinates, so the auton starts with setPose().
 */
void pathExample() {
  setPose(-36, -60, 0);
  Path out = pathFromPoints({{-36, -60}, {-36, -32}, {-12, -22}, {12, -30}, {34, -24}});
  followPath(out, 5000);
  Path back = pathFromPoints({{34, -24}, {12, -30}, {-12, -22}, {-36, -32}, {-36, -52}});
  followPathRamsete(back, 5000, true);
}

/*
 * profiledExample
 * Straight drives and turns with motion profiles: they speed up and slow down smoothly.
 */
void profiledExample() {
  driveProfiled(36, 3000);
  turnProfiled(90, 2000);
  driveProfiled(24, 3000);
  turnProfiled(0, 2000);
  driveProfiled(-36, 3000);
}


// ============================================================================
// PHskSE 1 TESTS (checking the robot's measurements before tuning PID)
// ============================================================================

// Shows where RW thinks the robot is on the controller screen until you press B
void showPoseUntilB() {
  stopChassis(pros::MotorBrake::coast);
  while (!controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    controller_1.print(1, 0, "x%6.1f y%6.1f   ", x_pos, y_pos);
    pros::delay(60);
    controller_1.print(2, 0, "h%6.1f  B=done   ", getInertialHeading());
    pros::delay(60);
  }
}


// Put the robot at myAuton's start, then push it by hand and read x and y off the controller
// to use in moveToPoint(). Pushing forward should raise y and turning right should raise h.
// If y goes down, reverse the vertical tracker port. If y doesn't change, check its port and wiring
void poseCheck() {
  setPose(my_start_x, my_start_y, my_start_heading);
  showPoseUntilB();
}

// Checks the distance sensor offsets. Put the robot at myAuton's start (tape-measure it), run this,
// and read the Brain screen: every sensor that sees a wall should show an error close to 0.
// A sensor that's off by the same amount everywhere has a wrong offset along the way it faces
void distanceCheck() {
  setPose(my_start_x, my_start_y, my_start_heading);
  showDistanceSensorsUntilB();
}


// Spin the robot by hand 10 full turns clockwise, then read imu_scale off the Brain screen.
// Never ends: restart the program when done
void imuScaleTest() {
  measureImuScale(10);
}

// ============================================================================
// PHASE 2 TESTS (PID tuning)
// Each move's time and final error show on the Brain screen, the worst ones on the controller.
// Error is measured 250 msec after the move ends, so overshoot that happens after it exits counts
// ============================================================================

// Shows the summary on the controller until you press B
void showTuningSummary(const char* name, double worst_error, double total_sec, int moves) {
  stopChassis(pros::MotorBrake::coast);
  while (!controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    controller_1.print(1, 0, "%s max err %.1f   ", name, worst_error);
    pros::delay(60);
    controller_1.print(2, 0, "avg %.2fs B=done   ", total_sec / moves);
    pros::delay(60);
  }
}

// Max voltage for the Turn Tune turns. Most skills turns run at 4.5-6.5 V, so tune at that speed
static const double turn_test_volts = 6;

// Overshoot tracking for turnTuningTest: a background task watches how far past the target the
// heading swings while turnToAngle() runs
static bool turn_sampling = false;
static double turn_sample_target = 0, turn_sample_dir = 1, turn_sample_peak = 0;

// Turns of different sizes (the robot ends where it started). Tune turn_kp / turn_kd with this.
// Each row: how long the turn took, the most it swung past the target, and where it ended up
void turnTuningTest() {
  auto_reset_enabled = false;
  setPose(0, 0, 0);
  hw::screenClear(0x000000);
  const double targets[] = {90, 0, 45, 0, 180, 0, 130, 0, -90, 0};
  const int count = sizeof(targets) / sizeof(targets[0]);
  double worst = 0, worst_over = 0, total_sec = 0;
  double secs[count], overs[count], errors[count];
  for (int i = 0; i < count; i++) {
    double from = getInertialHeading();
    turn_sample_target = targets[i];
    turn_sample_dir = std::remainder(targets[i] - from, 360.0) >= 0 ? 1 : -1;
    turn_sample_peak = 0;
    turn_sampling = true;
    pros::Task([] {
      while (turn_sampling) {
        double past = turn_sample_dir * std::remainder(getInertialHeading() - turn_sample_target, 360.0);
        turn_sample_peak = std::fmax(turn_sample_peak, past);
        pros::delay(10);
      }
    });

    double start = pros::millis();
    turnToAngle(targets[i], 2000, true, turn_test_volts);
    double sec = (pros::millis() - start) / 1000.0;
    pros::delay(250);  // Overshoot after the turn exits still counts
    turn_sampling = false;
    pros::delay(20);

    double error = std::remainder(targets[i] - getInertialHeading(), 360.0);
    total_sec += sec;
    worst = std::fmax(worst, std::fabs(error));
    worst_over = std::fmax(worst_over, turn_sample_peak);
    secs[i] = sec;
    overs[i] = turn_sample_peak;
    errors[i] = error;
    printf("turn to %.0f: %.2f s, overshoot %.2f, error %.2f deg\n", targets[i], sec, turn_sample_peak, error);
  }
  // turnToAngle() draws its graph over the screen, so show every turn once they're all done
  hw::screenClear(0x000000);
  for (int i = 0; i < count; i++) {
    hw::screenRow(i + 1, "To %4.0f  %.2f s  over %4.1f  end err %5.1f", targets[i], secs[i], overs[i], errors[i]);
  }
  hw::screenRow(11,"Worst overshoot %.1f deg   worst end error %.1f deg", worst_over, worst);
  showTuningSummary("TURN", worst, total_sec, count);
}

// Max voltage for the Drive Tune moves. Most skills drives run at 5-7 V, so tune at that speed
static const double drive_test_volts = 7;

// Overshoot tracking for driveTuningTest, like the turn test
static bool drive_sampling = false;
static double drive_sample_target = 0, drive_sample_dir = 1, drive_sample_peak = 0;

// Short forward and back drives with moveToPoint() (odometry, the same as skills), ending where it
// started. Tune distance_kp / distance_kd with this. Needs about 3 feet of room in front of the robot
void driveTuningTest() {
  auto_reset_enabled = false;
  setPose(0, 0, 0);
  hw::screenClear(0x000000);
  const double targets[] = {24, 0, 12, 0, 36, 0, 6, 0};  // y to drive to, in inches
  const int count = sizeof(targets) / sizeof(targets[0]);
  double worst = 0, worst_over = 0, total_sec = 0;
  double secs[count], overs[count], errors[count], headings[count];
  for (int i = 0; i < count; i++) {
    int dir = targets[i] > y_pos ? 1 : -1;  // Forward to go up, backward to come back
    drive_sample_target = targets[i];
    drive_sample_dir = dir;
    drive_sample_peak = 0;
    drive_sampling = true;
    pros::Task([] {
      while (drive_sampling) {
        drive_sample_peak = std::fmax(drive_sample_peak, drive_sample_dir * (y_pos - drive_sample_target));
        pros::delay(10);
      }
    });

    double start = pros::millis();
    moveToPoint(0, targets[i], dir, 2500, true, drive_test_volts);
    double sec = (pros::millis() - start) / 1000.0;
    pros::delay(250);  // Overshoot after the move exits still counts
    drive_sampling = false;
    pros::delay(20);

    double error = targets[i] - y_pos;
    total_sec += sec;
    worst = std::fmax(worst, std::fabs(error));
    worst_over = std::fmax(worst_over, drive_sample_peak);
    secs[i] = sec;
    overs[i] = drive_sample_peak;
    errors[i] = error;
    headings[i] = getInertialHeading();
    printf("drive to y %.0f: %.2f s, overshoot %.2f, error %.2f in\n", targets[i], sec, drive_sample_peak, error);
  }
  // The motions may draw over the screen, so show every drive once they're all done
  hw::screenClear(0x000000);
  for (int i = 0; i < count; i++) {
    hw::screenRow(i + 1, "To y %3.0f  %.2f s  over %4.1f  err %5.1f  h %5.1f", targets[i], secs[i], overs[i], errors[i],
                  headings[i]);
  }
  hw::screenRow(10, "Worst overshoot %.1f in   worst end error %.1f in", worst_over, worst);
  showTuningSummary("DRIVE", worst, total_sec, count);
}
