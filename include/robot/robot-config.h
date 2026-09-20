#pragma once

#include "main.h"
#include "lemlib/api.hpp"

extern pros::Controller controller;
extern pros::Controller partner;

extern pros::MotorGroup left_mg;
extern pros::MotorGroup right_mg;

extern pros::MotorGroup lift;
extern pros::Motor intake;
extern pros::MotorGroup toggles;

extern pros::adi::DigitalOut claw;
extern pros::adi::DigitalOut two_bar;
extern pros::adi::DigitalOut brake_1;
extern pros::adi::DigitalOut brake_2;
extern pros::adi::DigitalOut toggle_1;
extern pros::adi::DigitalOut toggle_2;

extern pros::Imu imu;
extern pros::AIVision aivision_back;
extern pros::AIVision aivision_side;
extern pros::ADIAnalogIn potentiometer;