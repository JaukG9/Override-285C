#include "main.h"
#include "lemlib/api.hpp"
#include "robot/drivetrain.h"
#include "robot/robot-config.h"
#include "robot/odometry.h"
#include "robot/functions.h"

bool right_arrow, L1, L2, R2, A = false;

/**
 * @brief get controller inputs
 * 
 * Take contrller inputs and modify variables for use in main.
 */
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

/**
 * @brief translates controller input to drivetrain movements
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

/**
 * @brief get rotation
 * @return the rotation sensor's value in deg
 * 
 * Takes the rotation value of the rotation sensor.
 */
double get_rotation(){
    return rotation.get_position() / 100.0;
}

/**
 * @brief gets lift motor pos (deg)
 * @return the position of the lift motors
 * 
 * Avg. position of the lift motors in degrees (0 at bottom, + = up).
 */
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


/**
 * @brief get lift height
 * @return the height in inches
 * 
 * Translate lift deg to height in inches.
 */
double get_lift_height(){
    return lift_deg_to_height(get_lift_motor_deg());
}

/**
 * @brief set lift to a designated height
 * @param[in] height the designated height to lift to
 * 
 * Set target for the lift PID to the inputted height.
 */
void lift_pos(double height){
    do_lift_PID = true;
    target = std::clamp(height, 0.0, LIFT_MAX_HEIGHT);
}

/**
 * @brief manually run lift
 * @param[in] up whether to lift up (true) or down (false)
 * 
 * Sets the PID target to a value up or down of the current one, to move manually.
 */
void lift_manual(bool up){
    double height = get_lift_height();
    if(height < 5 && !up){
        lift.move_velocity(-60);
    }else{lift.move_velocity(up ? 125 : -120);}
}

bool claw_closed = false;

/**
 * @brief set claw
 * 
 * Open or close the claw.
 */
void set_claw(bool closed){
    claw_closed = closed;
}

/**
 * @brief async set claw
 * 
 * Open or close the claw in the background.
 */
void async_claw(void*){
    if(claw_closed){
        claw.move_velocity(-200);
    }else{
        claw.move_velocity(200);
    }
    pros::delay(500);
    claw.brake();
}