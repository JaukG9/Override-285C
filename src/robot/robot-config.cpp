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
pros::MotorGroup left_mg({16, 17}, pros::MotorGearset::blue);       // left DT motors
pros::MotorGroup right_mg({-19, -20}, pros::MotorGearset::blue);    // right DT motors

pros::MotorGroup lift({11, -18}, pros::MotorGearset::green);        // lift motors
// pros::MotorGroup toggles({-1, 2}, pros::MotorGearset::green);       // toggle motors
// pros::Motor intake(15, pros::MotorGearset::blue);                   // intake motor

pros::Motor claw(-1, pros::MotorGearset::green);

/* init pistons */
pros::adi::DigitalOut claw_rotate('B');
pros::adi::DigitalOut two_bar('C');
pros::adi::DigitalOut brake_1('D');
pros::adi::DigitalOut brake_2('E');
pros::adi::DigitalOut toggle_1('F');
pros::adi::DigitalOut toggle_2('G');

/* init sensors */
pros::AIVision aivision_back(3);
pros::AIVision aivision_side(8);
pros::Rotation rotation(4);

/* global variables */
