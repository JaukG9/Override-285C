#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"

/* TODO: 
    SENSORS:
    - 2 AI sensors: back, side --> 3, 8
    - 2 distance sensors: front, side --> 4, 9
    - 1 rotation sensor: horizontal tracking wheel --> 13
    - 1 IMU --> 18
    - 1 optical sensor: color sort (T) --> 14
    - 1 potentiometer: lift --> 10
    
    MOTORS:
    - 4x 11W: drive --> 19, 20 (left); 16, 17 (right)
    - 2x 11W: lift --> 11 (left); 12 (right)
    - 1x 11W: intake --> 15
    - 2x 5.5W: toggle on each side --> 1 (left), 2 (right)

    PISTONS:
    - 1x double-acting: claw --> A
    - 1x single-acting: 2-bar --> B
    - 2x single-acting: brake --> C, D
    - 2x single-acting: toggles --> E, F
*/

/* init controllers */
pros::Controller controller(pros::E_CONTROLLER_MASTER);
pros::Controller partner(pros::E_CONTROLLER_PARTNER);

/* init motors */
pros::MotorGroup left_mg({-19, -20}, pros::MotorGearset::blue); // left DT motors
pros::MotorGroup right_mg({16, 17}, pros::MotorGearset::blue);  // right DT motors

pros::MotorGroup lift({-11, 12}, pros::MotorGearset::blue);     // lift motors
pros::Motor intake(15, pros::MotorGearset::blue);               // intake motor
pros::MotorGroup toggles({-1, 2}, pros::MotorGearset::green);   // toggle motors

/* init pistons */
pros::adi::DigitalOut claw('A');
pros::adi::DigitalOut two_bar('B');
pros::adi::DigitalOut brake_1('C');
pros::adi::DigitalOut brake_2('D');
pros::adi::DigitalOut toggle_1('E');
pros::adi::DigitalOut toggle_2('F');

/* init sensors */


/* global variables */
