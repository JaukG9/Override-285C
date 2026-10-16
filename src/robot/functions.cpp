#include "main.h"
#include "lemlib/api.hpp"
#include "robot/drivetrain.h"
#include "robot/robot-config.h"
#include "robot/odometry.h"
#include "robot/functions.h"

bool right_arrow, L1, L2, R2, A = false;
void get_inputs(void*){
    while(true){
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)){
            right_arrow = true;
        }else{right_arrow = false;}
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
            L1 = true;
        }else{L1 = false;}
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
            L2 = true;
        }else{L2 = false;}
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
            R2 = true;
        }else{R2 = false;}
        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
            A = true;
        }else{A = false;}
        pros::delay(10);
    }
}

void score(int height){

}

/**
 * @brief Translates controller input to drivetrain movements
 *
 * This function detects vertical movements on the joysticks of the controller, and
 * based on how much each joystick is moved up or down, supplies voltage to the drivetrain
 * motors of the corresponding side. The left side of the drive train is controlled by the
 * left joystick, and the right side of the drive train is controlled by the right joystick.
 * Down reverses the wheels, and up moves them forward. When the joysticks have unequal
 * magnitudes, the robot will turn in the direction of the one with the greater magnitude.
 * The vertical directional buttons on the controller can be used to move the drivetrain
 * at maximum speed in the forward (up arrow) or backwards (down arrow) direction.
 */
void tank(){
    int axisL = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);  // Gets amount forward/backward from left joystick
    int axisR = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y); // Gets the turn left/right from right joystick
    
    // Checks if either of the vertical directional buttons are being pressed
    if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)){
        axisL = 127 * .85;
        axisR = 127 * .85;
    }else if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)){
        axisL = -127 * .85;
        axisR = -127 * .85;
    }

    chassis.tank(axisL, axisR); // tank drive
}

// degrees. the rotation sensor reports centidegrees (36000 per turn), so divide by 100
double get_rotation(){
    return rotation.get_position() / 100.0;
}

// average position of the lift motors in degrees (0 at the bottom, + = up).
// NAN if neither motor is responding
double get_lift_motor_deg(){
    double total = 0;
    int count = 0;
    for(double pos : lift.get_position_all()){
        if(std::isfinite(pos) && pos != PROS_ERR_F){
            total += pos;
            count++;
        }
    }
    if(count == 0) return NAN;
    return lift_motor_dir * total / count;
}

// lift height in inches, from the lift motor encoders
double get_lift_height(){
    return lift_deg_to_height(get_lift_motor_deg());
}

// height in inches
void lift_pos(double height){
    do_lift_PID = true;
    target = std::clamp(height, 0.0, LIFT_MAX_HEIGHT);
}

void lift_manual(bool up){
    if(!do_lift_PID){
        target = get_lift_height();
    }

    double height = get_lift_height();
    if(up){
        lift_pos(std::min(target + 4.5, height + 9));
    }else{
        lift_pos(std::max(target - 3, height - 8));
    }
}