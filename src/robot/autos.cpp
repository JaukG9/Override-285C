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
    // activate flipouts and toggle twice
    chassis.setPose(-66, 0, 270);
    lift_pos(30);
    chassis.moveToPoint(-56, 0, 700, {.forwards = false});
    lift_pos(17);
    chassis.moveToPoint(-76, 0, 700);
    lift_pos(10);
    chassis.moveToPoint(-56, 0, 700, {.forwards = false});
    lift_pos(5);
    chassis.moveToPoint(-76, 0, 700);
    lift_pos(2);
}

/**
 * @brief runs the programming skills autonomous.
 *
 * Simple Override Skills Route
 */
void skills(){
    // activate flipouts and toggle twice
    chassis.setPose(-66, 0, 270);
    lift_pos(30);
    chassis.moveToPoint(-56, 0, 700, {.forwards = false});
    lift_pos(17);
    chassis.moveToPoint(-76, 0, 700);
    lift_pos(10);
    chassis.moveToPoint(-56, 0, 700, {.forwards = false});
    lift_pos(5);
    chassis.moveToPoint(-76, 0, 700);
    lift_pos(2);

    // score the 1st pin
    chassis.moveToPoint(-48, 0, 1000, {.forwards = false});
    lift_pos(2);
    chassis.turnToHeading(180, 750);
    chassis.moveToPoint(-48, -20, 1000);
    chassis.waitUntilDone();
    lift_pos(0);
    pros::delay(120);
    claw.set_value(false);

    // score the 2nd pin
    chassis.moveToPoint(-48, -12, 1000, {.forwards = false});
    chassis.turnToHeading(120, 750);
    chassis.moveToPoint(-29.25, -20, 1000);
    chassis.moveToPoint(-25.6, -22, 1000, {.maxSpeed = 35});
    pros::delay(500);
    claw.set_value(true);
    chassis.turnToHeading(270, 800);
    lift_pos(10);
    chassis.moveToPoint(-44, -24, 1000);
    chassis.waitUntilDone();
    lift_pos(3);
    pros::delay(200);
    claw.set_value(false);

    // score the 3rd pin
    chassis.moveToPoint(-36, -24, 1000, {.forwards = false});
    chassis.turnToHeading(120, 800);
    lift_pos(0);
    chassis.moveToPoint(-44.2, -41.7, 1000);
    chassis.moveToPoint(-45.6, -44.8, 1000, {.maxSpeed = 35});
    pros::delay(500);
    claw.set_value(true);
    chassis.turnToHeading(0, 800);
    lift_pos(17);
    chassis.moveToPoint(-48, -27, 1000);
    chassis.waitUntilDone();
    lift_pos(10);
    pros::delay(200);
    claw.set_value(false);

    // matchload and score [1]
    chassis.moveToPoint(-48, -60, 1200, {.forwards = false});
    chassis.turnToHeading(270, 800);
    lift_pos(0);
    chassis.moveToPoint(-66, -60, 1000);
    chassis.waitUntilDone();
    claw.set_value(true);
    chassis.moveToPoint(-48, -60, 1000, {.forwards = false});
    chassis.turnToHeading(0, 800);
    lift_pos(24);
    chassis.moveToPoint(-48, -27, 1200);
    chassis.waitUntilDone();
    lift_pos(18);
    pros::delay(200);
    claw.set_value(false);

    // matchload and score [2]
    chassis.moveToPoint(-48, -60, 1200, {.forwards = false});
    chassis.turnToHeading(270, 800);
    lift_pos(0);
    chassis.moveToPoint(-66, -60, 1000);
    chassis.waitUntilDone();
    claw.set_value(true);
    chassis.moveToPoint(-48, -60, 1000, {.forwards = false});
    chassis.turnToHeading(0, 800);
    lift_pos(33);
    chassis.moveToPoint(-48, -27, 1200, {.maxSpeed = 90});
    chassis.waitUntilDone();
    lift_pos(25);
    pros::delay(200);
    claw.set_value(false);
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
 * @brief 3 pins + toggle
 * 
 * Turn the toggle twice (to our color) then scores 3 pins and
 * moves to matchloader.
 */
void close_3_rush(){
    // toggle twice
    chassis.setPose(-61, 3, 270);
    lift_pos(30);
    chassis.moveToPoint(-54, 3, 1500, {.forwards = false, .maxSpeed = 70});
    chassis.waitUntilDone();
    lift_pos(16);
    pros::delay(500);
    chassis.moveToPoint(-72, 3, 700);
    chassis.moveToPoint(-54, 3, 700, {.forwards = false});
    lift_pos(10);
    chassis.moveToPoint(-72, 3, 700);
    lift_pos(5);

    // score the 1st pin
    chassis.moveToPoint(-48, 3, 1000, {.forwards = false});
    lift_pos(2);
    chassis.turnToHeading(180, 750);
    chassis.moveToPoint(-48, -28, 1500, {.maxSpeed = 70});
    chassis.waitUntilDone();
    lift_pos(0);
    chassis.moveToPoint(-48, -23, 300, {.maxSpeed = 100});
    pros::delay(400);
    claw.set_value(false);

    // score the 2nd pin
    chassis.moveToPoint(-48, 2, 750, {.forwards = false});
    chassis.turnToHeading(135, 750);
    chassis.moveToPoint(-30.5, -19, 1000);
    chassis.moveToPoint(-21, -28.5, 900, {.maxSpeed = 35});
    pros::delay(500);
    claw.set_value(true);
    chassis.turnToHeading(270, 800);
    lift_pos(10);
    chassis.moveToPoint(-48, -27.5, 1000);
    chassis.waitUntilDone();
    lift_pos(3);
    pros::delay(200);
    claw.set_value(false);

    // score the 3rd pin
    chassis.moveToPoint(-26, -26, 1000, {.forwards = false});
    lift_pos(0);
    chassis.turnToHeading(225, 750);
    chassis.moveToPoint(-44.5, -45.5, 1000);
    chassis.moveToPoint(-55, -56, 750, {.maxSpeed = 30});
    pros::delay(400);
    claw.set_value(true);
    chassis.turnToHeading(0, 750);
    lift_pos(18);
    chassis.moveToPoint(-50, -24, 1000);
    pros::delay(200);
    claw.set_value(false);

    // go to matchloader
    chassis.moveToPoint(-48, -33, 700, {.forwards = false});
    chassis.turnToHeading(215, 750);
    chassis.moveToPoint(-60, -55, 1000);
}

/**
 * @brief 3 pins + toggle
 * 
 * Turn the toggle twice (to our color) then scores 3 pins and
 * moves to matchloader.
 */
void far_3_rush(){
    // toggle twice
    chassis.setPose(-61, -3, 270);
    lift_pos(25);
    chassis.moveToPoint(-54, -3, 700, {.forwards = false});
    chassis.moveToPoint(-72, -3, 700);
    lift_pos(17);
    chassis.moveToPoint(-54, -3, 700, {.forwards = false});
    lift_pos(10);
    chassis.moveToPoint(-72, -3, 700);
    lift_pos(5);

    // score the 1st pin
    chassis.moveToPoint(-48, -3, 1000, {.forwards = false});
    lift_pos(2);
    chassis.turnToHeading(0, 750);
    chassis.moveToPoint(-48, 28, 1000);
    chassis.waitUntilDone();
    lift_pos(0);
    pros::delay(120);
    claw.set_value(false);

    // score the 2nd pin
    chassis.moveToPoint(-48, -2, 750, {.forwards = false});
    chassis.turnToHeading(45, 750);
    chassis.moveToPoint(-30.5, 19, 1000);
    chassis.moveToPoint(-21, 28.5, 900, {.maxSpeed = 35});
    pros::delay(500);
    claw.set_value(true);
    chassis.turnToHeading(270, 800);
    lift_pos(10);
    chassis.moveToPoint(-48, 27.5, 1000);
    chassis.waitUntilDone();
    lift_pos(3);
    pros::delay(200);
    claw.set_value(false);

    // score the 3rd pin
    chassis.moveToPoint(-26, 26, 1000, {.forwards = false});
    lift_pos(0);
    chassis.turnToHeading(315, 750);
    chassis.moveToPoint(-44.5, 45.5, 1000);
    chassis.moveToPoint(-55, 56, 750, {.maxSpeed = 30});
    pros::delay(400);
    claw.set_value(true);
    chassis.turnToHeading(180, 750);
    lift_pos(18);
    chassis.moveToPoint(-50, 24, 1000);
    pros::delay(200);
    claw.set_value(false);

    // go to matchloader
    chassis.moveToPoint(-48, 33, 700, {.forwards = false});
    chassis.turnToHeading(325, 750);
    chassis.moveToPoint(-60, 55, 1000);
}

/**
 * @brief 2 pins on 2 goals from wall
 * 
 * Score preload and the two wall pins (one per alliance side goal).
 */
void far_wall_4(){
    // activate flipouts and toggle twice
    chassis.setPose(0, -66, 180);
    lift_pos(30);
    chassis.moveToPoint(0, -56, 700, {.forwards = false});
    lift_pos(17);
    chassis.moveToPoint(0, -76, 700);
    lift_pos(10);
    chassis.moveToPoint(0, -56, 700, {.forwards = false});
    lift_pos(5);
    chassis.moveToPoint(0, -76, 700);
    lift_pos(2);

    // score the preload
    chassis.moveToPoint(0, -48, 1000, {.forwards = false});
    chassis.turnToHeading(270, 750);
    chassis.moveToPoint(-22, -48, 1000);
    chassis.waitUntilDone();
    lift_pos(0);
    pros::delay(200);
    claw.set_value(false);

    // pick up and score the right wall pin
    chassis.moveToPose(0, -56, 310, 1000, {.forwards = false});
    chassis.turnToHeading(110, 800);
    chassis.moveToPose(21, -64, 150, 1000);
    chassis.waitUntilDone();
    claw.set_value(true);
    chassis.turnToHeading(0, 800);
    lift_pos(12);
    chassis.moveToPoint(24, -50, 1000);
    chassis.waitUntilDone();
    lift_pos(8);
    pros::delay(200);
    claw.set_value(false);

    // pick up and score the left wall pin
    chassis.moveToPose(0, -56, 90, 1000, {.forwards = false});
    chassis.turnToHeading(250, 800);
    chassis.moveToPose(-21, -64, 210, 1000);
    chassis.waitUntilDone();
    claw.set_value(true);
    chassis.turnToHeading(0, 800);
    lift_pos(10);
    chassis.moveToPoint(-24, -50, 1000);
    chassis.waitUntilDone();
    lift_pos(6);
    pros::delay(200);
    claw.set_value(false);
};

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
        Button(10, 10, 225, 105, "Left Rush", pros::Color::peach_puff, pros::Color::black),
        Button(245, 10, 225, 105, "", pros::Color::peach_puff, pros::Color::black),
        Button(10, 125, 225, 105, "", pros::Color::peach_puff, pros::Color::black),
        Button(245, 125, 225, 105, "", pros::Color::peach_puff, pros::Color::black)
    };
    static Button rightAutons[] = {
        Button(10, 10, 225, 105, "Right Rush", pros::Color::peach_puff, pros::Color::black),
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