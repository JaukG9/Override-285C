#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"

void tank();

void score(int numStacked);

double get_rotation();
double get_lift_height();
void lift_pos(double height);
void lift_manual(bool up);

extern bool right_arrow, L1, L2, R2, A;
void get_inputs(void*);