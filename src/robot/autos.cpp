#include "main.h"
#include "lemlib/api.hpp"
#include "robot/odometry.h"
#include "robot/functions.h"
#include "robot/autos.h"

/**
 * @brief runs the skip autonomous.
 *
 * 
 */
void skip(){
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 5, 1000);
}

/**
 * @brief runs the programming skills autonomous.
 *
 * 
 */
void skills(){
    // 1st loop
    chassis.setPose(64.5, -46.35, 270);
    lift_pos(1.5);
    chassis.moveToPoint(48, -46.35, 1000);
    chassis.turnToHeading(0, 600);
    chassis.moveToPoint(48, -26, 1200);
    chassis.waitUntilDone();
    claw.set_value(false);
    pros::delay(250);

    // other loops
    for(int i = 0; i < 5; i++){
        chassis.moveToPoint(48, -56, 1500, {.forwards = false});
        chassis.turnToHeading(90, 800);
        chassis.moveToPoint(64, -56, 1000);
        lift_pos(0);
        chassis.waitUntilDone();
        claw.set_value(true);
        pros::delay(150);
        lift.brake();

        lift_pos(2);
        chassis.moveToPoint(48, -56, 1200, {.forwards = false});
        pros::delay(150);
        lift.brake();
        chassis.turnToHeading(0, 800);
        lift_pos(10 + 8 * i);
        chassis.moveToPoint(48, -26, 1200);
        pros::delay(200);
        lift.brake();
        chassis.waitUntilDone();
        lift_pos(4 + 8 * i);
        pros::delay(200);
        claw.set_value(false);
    }
}

/**
 * @brief runs the left repeater autonomous routine
 * 
 * Scores 3 pins from the initial pin, and two matchloaded pins.
 */
void left_repeater_route(){
    // 1st loop
    chassis.setPose(64.5, -46.35, 270);
    lift_pos(1.5);
    chassis.moveToPoint(48, -46.35, 1000);
    chassis.turnToHeading(0, 600);
    chassis.moveToPoint(48, -26, 1200);
    chassis.waitUntilDone();
    claw.set_value(false);
    pros::delay(250);

    // 2nd loop
    chassis.moveToPoint(48, -56, 1500, {.forwards = false});
    chassis.turnToHeading(90, 800);
    chassis.moveToPoint(64, -56, 1000);
    lift_pos(0);
    chassis.waitUntilDone();
    claw.set_value(true);
    pros::delay(150);
    lift.brake();

    lift_pos(2);
    chassis.moveToPoint(48, -56, 1200, {.forwards = false});
    pros::delay(150);
    lift.brake();
    chassis.turnToHeading(0, 800);
    lift_pos(10);
    chassis.moveToPoint(48, -26, 1200);
    pros::delay(200);
    lift.brake();
    chassis.waitUntilDone();
    lift_pos(3);
    pros::delay(100);
    claw.set_value(false);

    // 3rd loop
    chassis.moveToPoint(48, -56, 1500, {.forwards = false});
    lift_pos(0);
    chassis.turnToHeading(90, 800);
    chassis.moveToPoint(64, -56, 1200);
    chassis.waitUntilDone();
    claw.set_value(true);
    pros::delay(200);
    lift.brake();

    lift_pos(2);
    chassis.moveToPoint(48, -56, 900, {.forwards = false});
    pros::delay(150);
    lift.brake();
    chassis.turnToHeading(0, 600);
    lift_pos(18);
    chassis.moveToPoint(48, -26, 900);
    chassis.waitUntilDone();
    pros::delay(100);
    claw.set_value(false);
}

/**
 * @brief runs the left repeater autonomous routine
 * 
 * Scores 3 pins from the initial pin, and two matchloaded pins.
 */
void right_repeater_route(){
    // 1st loop
    chassis.setPose(61.5, 56, 270);
    chassis.moveToPoint(48, 56, 1000);
    chassis.turnToHeading(180, 600);
    chassis.moveToPoint(48, 26, 1200);
    chassis.waitUntilDone();
    claw.set_value(false);
    pros::delay(250);

    // 2nd loop
    chassis.moveToPoint(48, 56, 1500, {.forwards = false});
    chassis.turnToHeading(90, 800);
    chassis.moveToPoint(64, 56, 900);
    lift_pos(0);
    chassis.waitUntilDone();
    claw.set_value(true);
    pros::delay(150);
    lift.brake();

    lift_pos(2);
    chassis.moveToPoint(48, 56, 1200, {.forwards = false});
    pros::delay(150);
    lift.brake();
    chassis.turnToHeading(180, 800);
    lift_pos(10);
    chassis.moveToPoint(48, 26, 1200);
    pros::delay(200);
    lift.brake();
    chassis.waitUntilDone();
    lift_pos(3);
    pros::delay(100);
    claw.set_value(false);

    // 3rd loop
    chassis.moveToPoint(48, 56, 1500, {.forwards = false});
    lift_pos(0);
    chassis.turnToHeading(90, 800);
    chassis.moveToPoint(64, 56, 1200);
    chassis.waitUntilDone();
    claw.set_value(true);
    pros::delay(200);
    lift.brake();

    lift_pos(2);
    chassis.moveToPoint(48, 56, 900, {.forwards = false});
    pros::delay(150);
    lift.brake();
    chassis.turnToHeading(180, 600);
    lift_pos(18);
    chassis.moveToPoint(48, 26, 900);
    chassis.waitUntilDone();
    pros::delay(200);
    claw.set_value(false);
}



/**
 * @brief displays auton selector so user can select autos from one program
 *
 * Uses custom Button object to display 4 buttons to the screen where the user can
 * choose which auton they use. The first screen displays options like (1) Left,
 * (2) Right, (3) Skip, and (4) Skills. Options (3) and (4) choose their respective
 * autonomous routines. Options (1) and (2) display 4 more options which are as
 * follows (a) 4 Rush, (b) 7 Rush, (c) Split, (d) AWP.
 */
int autonomousSelection = 0;

void autonSelector(void*){
    int autonomousType = 0;
    autonomousSelection = -1;
    bool autonTypeSelected = false, autonSelected = false;

    Button autonType[] = {
        Button(10, 10, 225, 105, "Left", pros::Color::pale_violet_red, pros::Color::black),
        Button(245, 10, 225, 105, "Right", pros::Color::cyan, pros::Color::black),
        Button(10, 125, 225, 105, "Skip", pros::Color::white, pros::Color::black),
        Button(245, 125, 225, 105, "Skills", pros::Color::light_green, pros::Color::black)
    };

    pros::screen::set_pen(pros::Color::light_gray);
    pros::screen::fill_rect(0, 0, 480, 240);
    while(!autonTypeSelected){
        for(int i = 0; i < 4; i++){
            autonType[i].render();
            if(autonType[i].isClicked()){
                autonomousType = i;
                autonTypeSelected = true;
            }
        }
        pros::delay(20);
    }
    pros::screen::erase();

    Button* auton = nullptr;
    
    static Button leftAutons[] = {
        Button(10, 10, 225, 105, "Left Repeater", pros::Color::peach_puff, pros::Color::black),
        Button(245, 10, 225, 105, "", pros::Color::peach_puff, pros::Color::black),
        Button(10, 125, 225, 105, "", pros::Color::peach_puff, pros::Color::black),
        Button(245, 125, 225, 105, "", pros::Color::peach_puff, pros::Color::black)
    };
    static Button rightAutons[] = {
        Button(10, 10, 225, 105, "Right Repeater", pros::Color::peach_puff, pros::Color::black),
        Button(245, 10, 225, 105, "", pros::Color::peach_puff, pros::Color::black),
        Button(10, 125, 225, 105, "", pros::Color::peach_puff, pros::Color::black),
        Button(245, 125, 225, 105, "", pros::Color::peach_puff, pros::Color::black)
    };
    

    switch(autonomousType){
        case 0: auton = leftAutons; break;
        case 1: auton = rightAutons; break;
        case 2: autonomousSelection = 0; autonSelected = true; break;
        case 3: autonomousSelection = 9; autonSelected = true; break;
    }

    pros::screen::set_pen(pros::Color::dark_gray);
    while(!autonSelected && autonTypeSelected){
        for(int i = 0; i < 4; i++){
            if(auton[i].isClicked()){
                autonomousSelection = i * 2 + 1 + autonomousType;
                auton[i].buttonColor = pros::Color::medium_violet_red;
                autonSelected = true;
            }
            auton[i].render();
        }

        pros::delay(20);
    }
}