#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "robot/drivetrain.h"
#include "robot/odometry.h"

extern pros::Imu imu;

/* CUSTOM MOTION CHAINING */
// max deceleration (in/s^2 and deg/s^2)
const float CHAIN_LATERAL_ACCEL = 150;
const float CHAIN_ANGULAR_ACCEL = 1200;

// max carry speed when chaining
const float CHAIN_LATERAL_CARRY = 0.5;
const float CHAIN_ANGULAR_CARRY = 0.25;

// stop steering threshold
const float LEMLIB_SETTLE_RANGE = 7.5;

// how far off (degrees) a close point can be before chainToPoint turns to face it first
const float CHAIN_FACE_TOLERANCE = 10;

struct ChainToPointParams{
    bool forwards = true;
    int maxSpeed = 127;
    bool chain = true;
};
struct ChainToPoseParams{
    bool forwards = true;
    float horizontalDrift = 0;
    float lead = 0.6;
    int maxSpeed = 127;
    bool chain = true;
};
struct ChainToHeadingParams{
    lemlib::AngularDirection direction = lemlib::AngularDirection::AUTO;
    int maxSpeed = 127;
    bool chain = true;
};

class CustomChassis : public lemlib::Chassis{
    // wait for motion to end
    void waitForMotion(){
        while(isInMotion()) pros::delay(5);
    }

    // top speeds for DT (angular & lateral)
    float lateralTopSpeed(){
        return drivetrain.rpm / 60 * M_PI * drivetrain.wheelDiameter;
    }
    float angularTopSpeed(){
        return lateralTopSpeed() * 2 / drivetrain.trackWidth * 180 / M_PI;
    }

    // find minSpeed and earlyExitRange
    void chainValues(float distance, float topSpeed, float maxSpeed, float accel, float carry, float& minSpeed, float& earlyExitRange){
        float reachable = std::min(std::sqrt(2 * accel * distance), topSpeed * maxSpeed / 127);
        float carrySpeed = reachable * carry;
        minSpeed = carrySpeed / topSpeed * 127;
        earlyExitRange = std::min(carrySpeed * carrySpeed / (2 * accel), distance / 2);
    }

    public:
    using lemlib::Chassis::Chassis;

    /**
     * @brief motion chaining to point
     * 
     * Input x, y, and timeout + any additional params to get auto motion chaining
     */
    void chainToPoint(float x, float y, int timeout, ChainToPointParams params = {}, bool async = true){
        waitForMotion();
        lemlib::Pose start = getPose();
        float distance = std::hypot(x - start.x, y - start.y);

        // turn then move to short distances
        if(distance < 2 * LEMLIB_SETTLE_RANGE && distance > 0.5){
            float bearing = std::atan2(x - start.x, y - start.y) * 180 / M_PI;
            if(distance < LEMLIB_SETTLE_RANGE){
                params.forwards = std::abs(lemlib::angleError(bearing, start.theta, false)) <= 90;
            }
            float facing = params.forwards ? bearing : bearing + 180;

            if(std::abs(lemlib::angleError(facing, start.theta, false)) > CHAIN_FACE_TOLERANCE){
                chainToHeading(facing, timeout);
                waitForMotion();
                start = getPose();
                distance = std::hypot(x - start.x, y - start.y);
            }
        }

        float minSpeed = 0, earlyExitRange = 0;
        if(params.chain){
            chainValues(distance, lateralTopSpeed(), std::abs(params.maxSpeed), CHAIN_LATERAL_ACCEL, CHAIN_LATERAL_CARRY, minSpeed, earlyExitRange);
        }

        moveToPoint(x, y, timeout,
                    {.forwards = params.forwards,
                     .maxSpeed = (float) params.maxSpeed,
                     .minSpeed = minSpeed,
                     .earlyExitRange = earlyExitRange},
                    async);
    }

    /**
     * @brief motion chaining to point
     * 
     * Input x, y, theta, and timeout + any additional params to get auto motion chaining
     */
    void chainToPose(float x, float y, float theta, int timeout, ChainToPoseParams params = {}, bool async = true){
        waitForMotion();
        lemlib::Pose start = getPose();
        float distance = std::hypot(x - start.x, y - start.y);

        float minSpeed = 0, earlyExitRange = 0;
        if(params.chain){
            chainValues(distance, lateralTopSpeed(), std::abs(params.maxSpeed), CHAIN_LATERAL_ACCEL, CHAIN_LATERAL_CARRY, minSpeed, earlyExitRange);
        }

        moveToPose(x, y, theta, timeout,
                   {.forwards = params.forwards,
                    .horizontalDrift = params.horizontalDrift,
                    .lead = params.lead,
                    .maxSpeed = (float) params.maxSpeed,
                    .minSpeed = minSpeed,
                    .earlyExitRange = earlyExitRange},
                   async);
    }

    /**
     * @brief motion chaining to point
     * 
     * Input theta and timeout + any additional params to get auto motion chaining
     */
    void chainToHeading(float theta, int timeout, ChainToHeadingParams params = {}, bool async = true){
        waitForMotion();
        float error = lemlib::angleError(theta, getPose().theta, false, params.direction);
        float distance = std::abs(error);

        float minSpeed = 0, earlyExitRange = 0;
        if(params.chain){
            chainValues(distance, angularTopSpeed(), std::abs(params.maxSpeed), CHAIN_ANGULAR_ACCEL, CHAIN_ANGULAR_CARRY, minSpeed, earlyExitRange);
        }

        turnToHeading(theta, timeout,
                      {.direction = params.direction,
                       .maxSpeed = params.maxSpeed,
                       .minSpeed = (int) std::round(minSpeed),
                       .earlyExitRange = earlyExitRange},
                      async);
    }
};

extern CustomChassis chassis;

extern const double LIFT_MAX_HEIGHT;
extern const double LIFT_TOP_MOTOR_DEG;
double lift_deg_to_height(double deg);
extern int lift_motor_dir;
extern double target;
extern bool do_lift_PID;
void lift_PID(void*);