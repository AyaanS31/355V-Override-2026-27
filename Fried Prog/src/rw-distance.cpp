#include "rw-config.h"
#include "pros/error.h"
#include "rw/hardware.h"
#include <algorithm>
#include <cmath>

// ============================================================================
// DISTANCE SENSOR AUTO RESET
// A background task that keeps x_pos / y_pos lined up with the field walls using the four
// distance sensors. Start it once with startAutoReset() after startOdometry().
//
// Each sensor reading becomes an "error": how far odometry's X or Y is from what the wall
// says it should be. A sensor only corrects the position when:
//   1. Its reading is clean: sensor plugged in, offsets set, sees a wall in range, confident,
//      the beam hits the wall straight enough and away from the corners (RW's checks), and
//      away from anything that sits against the wall (auto_reset_wall_obstacles).
//   2. Its last auto_reset_samples errors in a row were all clean and agree with each other
//      (one bad reading restarts the count), so a robot or game element passing by is ignored.
//   3. The error is bigger than the deadband (no jitter) but smaller than the max (not a robot
//      or goal in front of the wall).
//   4. If two sensors both see walls on the same axis (e.g. front and back), they agree.
// The position then moves a fraction of the way (auto_reset_gain) toward the wall's answer each
// update, so corrections are smooth even in the middle of a motion.
//
// Readings lag behind the robot (distance_sensor_delay_msec), so each one is compared with where
// odometry said the robot was when the sensor measured it. That keeps it accurate while driving.
// Settings are in src/rw-config.cpp. Needs field-centered coordinates (see setPose()).
// ============================================================================

extern double field_half_size;  // RW's distance from the field center to each wall (inches)

bool auto_reset_running = false;
int auto_reset_count = 0;                      // Corrections applied since the program started
double auto_reset_last_x = 0, auto_reset_last_y = 0;  // Size of the last X and Y correction (inches)

namespace {

const int max_samples = 15;
const int history_size = 16;  // 160 msec of pose history for matching up delayed readings

struct SensorState {
  pros::Distance* sensor;
  double* offset_x;
  double* offset_y;
  double angle_deg;   // Direction the sensor faces on the robot (0 = front, 90 = right, ...)
  const char* name;
  int axis = 0;       // Which wall the current samples are against: 1 = X wall, 2 = Y wall
  double errors[max_samples];
  int count = 0;
  int next = 0;
  double last_reading_in = 0;

  void clear() { count = 0; next = 0; axis = 0; }
};

SensorState sensors[4] = {
  {&front_sensor, &front_sensor_offsetX, &front_sensor_offsetY, 0, "Front"},
  {&right_sensor, &right_sensor_offsetX, &right_sensor_offsetY, 90, "Right"},
  {&back_sensor, &back_sensor_offsetX, &back_sensor_offsetY, 180, "Back"},
  {&left_sensor, &left_sensor_offsetX, &left_sensor_offsetY, 270, "Left"},
};

int windowSize() { return std::clamp(auto_reset_samples, 1, max_samples); }

// Why a reading was skipped, for showDistanceSensorsUntilB()
enum class Skip { none, unplugged, no_offsets, no_wall, out_of_range, low_confidence, bad_angle, wall_obstacle, too_far_off };
const char* skipText(Skip skip) {
  switch (skip) {
    case Skip::none: return "ok";
    case Skip::unplugged: return "unplugged";
    case Skip::no_offsets: return "set offsets";
    case Skip::no_wall: return "no wall";
    case Skip::out_of_range: return "range";
    case Skip::low_confidence: return "low conf";
    case Skip::bad_angle: return "angle/corner";
    case Skip::wall_obstacle: return "wall object";
    case Skip::too_far_off: return "too far off";
  }
  return "";
}

// Works out one sensor's error against the wall, given where the robot was when it measured.
// Returns why it was skipped, or Skip::none with axis (1 = X, 2 = Y) and error filled in
Skip readError(SensorState& s, double robot_x, double robot_y, double heading_deg, int& axis, double& error) {
  if (!s.sensor->is_installed()) return Skip::unplugged;
  // Both offsets at 0 means they haven't been measured yet (no sensor face sits at the robot's center)
  if (*s.offset_x == 0 && *s.offset_y == 0) return Skip::no_offsets;

  int32_t mm = s.sensor->get();
  if (mm == PROS_ERR || mm <= 0 || mm >= 9999) return Skip::no_wall;
  double reading_in = mm / 25.4 * distance_sensor_scale;
  s.last_reading_in = reading_in;
  if (reading_in < auto_reset_min_range || reading_in > auto_reset_max_range) return Skip::out_of_range;
  // Confidence (0-63) only means something past 200 mm
  if (mm > 200 && s.sensor->get_confidence() < auto_reset_min_confidence) return Skip::low_confidence;

  double wall_pos = 0;
  axis = distanceResetPosition(robot_x, robot_y, heading_deg, *s.offset_x, *s.offset_y, s.angle_deg, reading_in,
                               field_half_size, distance_reset_max_angle, distance_reset_corner_margin, wall_pos);
  if (axis == 0) return Skip::bad_angle;

  // Where along the wall the beam lands. The beam is a cone, so it gets wider the further it goes
  double offset_x = 0, offset_y = 0;
  localToGlobal(*s.offset_x, *s.offset_y, degToRad(heading_deg), offset_x, offset_y);
  double beam_rad = degToRad(heading_deg + s.angle_deg);
  double along_wall = axis == 1 ? robot_y + offset_y + reading_in * std::cos(beam_rad)
                                : robot_x + offset_x + reading_in * std::sin(beam_rad);
  double clearance = auto_reset_obstacle_width / 2 + 1 + reading_in * std::tan(degToRad(auto_reset_beam_half_angle));
  for (double obstacle : auto_reset_wall_obstacles) {
    if (std::fabs(along_wall - obstacle) < clearance) return Skip::wall_obstacle;
  }

  error = wall_pos - (axis == 1 ? robot_x : robot_y);
  if (std::fabs(error) > auto_reset_max_error) return Skip::too_far_off;
  return Skip::none;
}

// Median of a sensor's samples, and how far apart they are (largest - smallest)
void windowStats(const SensorState& s, double& median, double& spread) {
  double sorted[max_samples];
  std::copy(s.errors, s.errors + s.count, sorted);
  std::sort(sorted, sorted + s.count);
  median = s.count % 2 ? sorted[s.count / 2] : (sorted[s.count / 2 - 1] + sorted[s.count / 2]) / 2;
  spread = sorted[s.count - 1] - sorted[0];
}

// Combines the sensors that see a wall on this axis into one error. Returns false if none is
// ready or two of them disagree (one is probably seeing a robot or game element)
bool axisError(int axis, double& error) {
  double weighted = 0, total_weight = 0;
  double lowest = 0, highest = 0;
  int ready = 0;
  for (SensorState& s : sensors) {
    if (s.axis != axis || s.count < windowSize()) continue;
    double median = 0, spread = 0;
    windowStats(s, median, spread);
    if (spread > auto_reset_max_spread) continue;
    // Closer walls give more exact readings, so they count for more
    double weight = 1.0 / std::fmax(s.last_reading_in, 4.0);
    weighted += median * weight;
    total_weight += weight;
    lowest = ready == 0 ? median : std::fmin(lowest, median);
    highest = ready == 0 ? median : std::fmax(highest, median);
    ready++;
  }
  if (ready == 0 || highest - lowest > auto_reset_max_disagree) return false;
  error = weighted / total_weight;
  return true;
}

void clearAll() {
  for (SensorState& s : sensors) s.clear();
}

}  // namespace

/*
 * runAutoReset
 * The auto reset task (see the top of this file). Start it with startAutoReset().
 */
void runAutoReset() {
  double history_x[history_size], history_y[history_size], history_heading[history_size];
  for (int i = 0; i < history_size; i++) {
    history_x[i] = x_pos;
    history_y[i] = y_pos;
    history_heading[i] = getInertialHeading();
  }
  double last_x = x_pos, last_y = y_pos, last_heading = getInertialHeading();
  auto_reset_running = true;
  int tick = 0;

  while (true) {
    double heading = getInertialHeading();

    // setPose() moves the pose further in one step than the robot can drive. Old samples and
    // history were against the old pose, so start over from the new one
    if (std::hypot(x_pos - last_x, y_pos - last_y) > 3 || std::fabs(heading - last_heading) > 30) {
      clearAll();
      for (int i = 0; i < history_size; i++) {
        history_x[i] = x_pos;
        history_y[i] = y_pos;
        history_heading[i] = heading;
      }
    }

    int now = tick % history_size;
    history_x[now] = x_pos;
    history_y[now] = y_pos;
    history_heading[now] = heading;

    // Distance sensors update about every 33 msec, so check them every 30
    if (tick % 3 == 0) {
      // MCL does the same job, so don't fight it if it's running
      bool steady = odom_speed_ips < auto_reset_max_speed && odom_turn_rate_dps < auto_reset_max_turn_rate;
      bool active = auto_reset_enabled && !mcl_running && steady;

      // Where the robot was when the sensors measured
      int delay_ticks = (int)std::clamp(std::round(distance_sensor_delay_msec / 10.0), 0.0, history_size - 1.0);
      int then = ((tick - delay_ticks) % history_size + history_size) % history_size;

      for (SensorState& s : sensors) {
        int axis = 0;
        double error = 0;
        if (!active || readError(s, history_x[then], history_y[then], history_heading[then], axis, error) != Skip::none) {
          s.clear();  // Samples have to be clean in a row
          continue;
        }
        if (axis != s.axis) {
          s.clear();  // The beam moved to a different wall (the robot turned)
          s.axis = axis;
        }
        s.errors[s.next] = error;
        s.next = (s.next + 1) % windowSize();
        s.count = std::min(s.count + 1, windowSize());
      }

      for (int axis = 1; axis <= 2; axis++) {
        double error = 0;
        if (!active || !axisError(axis, error) || std::fabs(error) < auto_reset_deadband) continue;
        double correction = error * std::clamp(auto_reset_gain, 0.0, 1.0);
        double* history = axis == 1 ? history_x : history_y;
        if (axis == 1) {
          x_pos += correction;
          auto_reset_last_x = correction;
        } else {
          y_pos += correction;
          auto_reset_last_y = correction;
        }
        // The stored samples and history were measured before this correction
        for (int i = 0; i < history_size; i++) history[i] += correction;
        for (SensorState& s : sensors) {
          if (s.axis != axis) continue;
          for (int i = 0; i < s.count; i++) s.errors[i] -= correction;
        }
        auto_reset_count++;
      }
    }

    last_x = x_pos;
    last_y = y_pos;
    last_heading = heading;
    tick++;
    pros::delay(10);
  }
}

/*
 * startAutoReset
 * Starts the auto reset task. Call it once, after startOdometry(). Turn it off for a while with
 * auto_reset_enabled = false (e.g. while pushing against a goal that's in front of a wall).
 */
void startAutoReset() {
  if (!auto_reset_running) pros::Task task(runAutoReset);
}

/*
 * drawDistanceSensors
 * Draws each sensor's reading and what it says the position is on the Brain screen.
 * Used by showDistanceSensorsUntilB() and the driver control sensor view in runDriver().
 */
void drawDistanceSensors() {
  double heading = getInertialHeading();
  for (int i = 0; i < 4; i++) {
    SensorState& s = sensors[i];
    int axis = 0;
    double error = 0;
    Skip skip = readError(s, x_pos, y_pos, heading, axis, error);
    int32_t mm = s.sensor->get();
    double reading_in = mm > 0 && mm < 9999 ? mm / 25.4 * distance_sensor_scale : 0;
    if (skip == Skip::none) {
      double wall_pos = (axis == 1 ? x_pos : y_pos) + error;
      hw::screenRow(i + 1, "%-5s %5.1f in  conf %2d  %c=%7.2f  error %5.2f", s.name, reading_in,
                    (int)s.sensor->get_confidence(), axis == 1 ? 'x' : 'y', wall_pos, error);
    } else {
      hw::screenRow(i + 1, "%-5s %5.1f in  conf %2d  skipped: %-12s", s.name, reading_in,
                    (int)s.sensor->get_confidence(), skipText(skip));
    }
  }
  hw::screenRow(6, "Pose x %7.2f  y %7.2f  h %6.1f", x_pos, y_pos, heading);
  hw::screenRow(7, "Auto resets %d  last x %+.2f  y %+.2f  %s   ", auto_reset_count, auto_reset_last_x,
                auto_reset_last_y, auto_reset_enabled ? "on" : "OFF");
}

/*
 * showDistanceSensorsUntilB
 * For checking the sensor offsets: shows each sensor's reading and what it says the position is
 * on the Brain screen, until you press B. Put the robot at a tape-measured spot first (setPose()).
 * With the right offsets, every sensor that sees a wall reads an error close to 0.
 */
void showDistanceSensorsUntilB() {
  stopChassis(pros::MotorBrake::coast);
  hw::screenClear(0x000000);
  while (!controller_1.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
    drawDistanceSensors();
    controller_1.print(1, 0, "x%6.1f y%6.1f   ", x_pos, y_pos);
    pros::delay(100);
  }
}