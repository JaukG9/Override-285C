#pragma once

#include "main.h"
#include "lemlib/api.hpp"

void set_lateral_gains(double kP, double kI, double kD);
void set_angular_gains(double kP, double kI, double kD);
void auto_angular_PID();
void auto_lateral_PID();
