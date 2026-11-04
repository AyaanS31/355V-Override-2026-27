#pragma once
#include "pros/adi.hpp"
#include "rw/api.h"

// Your own devices (besides the ones RW uses, which are in rw/devices.h), defined in src/rw-config.cpp
extern pros::Motor lift_right;
extern pros::Motor lift_right2;
extern pros::Motor lift_left;
extern pros::adi::Pneumatics clamp;
extern pros::adi::Pneumatics toggle;

// Distance sensor auto reset settings, defined in src/rw-config.cpp
extern bool auto_reset_enabled;
extern double auto_reset_min_range, auto_reset_max_range;
extern int auto_reset_min_confidence;
extern int auto_reset_samples;
extern double auto_reset_max_spread;
extern double auto_reset_deadband;
extern double auto_reset_max_error;
extern double auto_reset_max_disagree;
extern double auto_reset_gain;
extern double auto_reset_max_speed, auto_reset_max_turn_rate;
extern std::vector<double> auto_reset_wall_obstacles;
extern double auto_reset_obstacle_width;
extern double auto_reset_beam_half_angle;

// Distance sensor auto reset (src/rw-distance.cpp)
extern int auto_reset_count;
extern double auto_reset_last_x, auto_reset_last_y;
void startAutoReset();
void drawDistanceSensors();
void showDistanceSensorsUntilB();

// Your autonomous routines (src/rw-autons.cpp). Add each one to the selector in src/rw-user.cpp
void leftSide();
void rightSide();
void skills();
void asyncExample();
void pathExample();
void profiledExample();

// Test routines (src/rw-autons.cpp)
void poseCheck();
void distanceCheck();
void imuScaleTest();
void turnTuningTest();
void driveTuningTest();

// Controller pose display (src/rw-autons.cpp)
void printPoseLine();
void startPoseDisplay();
void stopPoseDisplay();
void setSkillsStartPose();
void printSkillsStartLine();

// Final position on the Brain screen when an auton ends (src/rw-autons.cpp)
extern bool auton_in_progress;
void showFinalPose();
void showLivePoseUntilAutonEnds();

// Called from src/main.cpp (src/rw-user.cpp)
void runPreAutonomous();
void runAutonomous();
void runDriver();
