#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"
#include "robot/drivetrain.h"

/* creation of drivetrain */
lemlib::Drivetrain drivetrain(&left_mg,                     // left motor group
                              &right_mg,                    // right motor group
                              10.882,                       // 10.882 inch track width
                              lemlib::Omniwheel::NEW_325,   // using new 3.25" omnis
                              360,                          // drivetrain rpm (360 = 600rpm * (36/60 gear ratio))
                              2                             // horizontal drift (2 for full omni DT)
);