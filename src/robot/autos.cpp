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
void skip(){}

/**
 * @brief runs the programming skills autonomous.
 *
 * 
 */
void skills(){}

/**
 * @brief runs the repeater autonomous routine
 * 
 * Scores 3 pins from the initial pin, and two matchloaded pins.
 */
void repeater_route(){
    // 1st loop
    chassis.setPose(60, -56, 340);
    chassis.moveToPoint(51, -33, 750);
    chassis.chainToPoint(53, -35, 750);
    chassis.chainToHeading(180, 750);
    chassis.chainToPose(56, -56, 90, 1000, {.chain = false});
    score(0);

    // 2nd loop
    chassis.resetChain();
    chassis.chainToHeading(340, 750);
    chassis.moveToPoint(51, -33, 750);
    chassis.chainToPoint(53, -35, 750);
    chassis.chainToHeading(180, 750);
    chassis.chainToPose(56, -56, 90, 1000, {.chain = false});
    score(1);

    // 3rd loop
    chassis.resetChain();
    chassis.chainToHeading(340, 750);
    chassis.moveToPoint(51, -33, 750);
    score(2);
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
        Button(10, 10, 225, 105, "", pros::Color::pale_violet_red, pros::Color::black),
        Button(245, 10, 225, 105, "", pros::Color::cyan, pros::Color::black),
        Button(10, 125, 225, 105, "", pros::Color::white, pros::Color::black),
        Button(245, 125, 225, 105, "", pros::Color::light_green, pros::Color::black)
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
        Button(10, 10, 225, 105, "", pros::Color::peach_puff, pros::Color::black),
        Button(245, 10, 225, 105, "", pros::Color::peach_puff, pros::Color::black),
        Button(10, 125, 225, 105, "", pros::Color::peach_puff, pros::Color::black),
        Button(245, 125, 225, 105, "", pros::Color::peach_puff, pros::Color::black)
    };
    static Button rightAutons[] = {
        Button(10, 10, 225, 105, "", pros::Color::peach_puff, pros::Color::black),
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