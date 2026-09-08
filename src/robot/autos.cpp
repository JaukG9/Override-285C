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
 * @brief displays auton selector so user can select autos from one program
 *
 * 
 */
int autonomousSelection = 0;

void autonSelector(void*){
    int autonomousType = 0;
    autonomousSelection = -1;
    bool autonTypeSelected = false, autonSelected = false;

    Button autonType[] = {

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