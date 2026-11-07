#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "robot/drivetrain.h"
#include "robot/odometry.h"

extern pros::Imu imu;

/* CUSTOM MOTION CHAINING (tuning values are in robot/chain.h) */
#include "robot/chain.h"

extern CustomChassis chassis;

extern const double LIFT_MAX_HEIGHT;
extern const double LIFT_TOP_MOTOR_DEG;
double lift_deg_to_height(double deg);
extern int lift_motor_dir;
extern double target;
extern bool do_lift_PID;
void lift_PID(void*);