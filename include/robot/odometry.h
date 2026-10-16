#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "robot/drivetrain.h"
#include "robot/odometry.h"

extern pros::Imu imu;

/* MOTION CHAINING
 * Chained motions don't stop at their target: they exit a little early while still moving, so
 * the next motion picks up without the robot stopping. How fast it's still going (minSpeed) and
 * how early it exits (earlyExitRange) come from the drive's top speed (from the drivetrain
 * settings) and the constants below, which are set for the override bot: 4 blue motors geared
 * 36:60 (360rpm) on 3.25" omnis. A 4 motor drive speeds up / slows down a lot slower than a 6
 * motor one, so it can't carry as much speed between motions.
 */
// how fast the drive can slow down (in/s^2 and deg/s^2)
const float CHAIN_LATERAL_ACCEL = 150;
const float CHAIN_ANGULAR_ACCEL = 1200;
// how much of the fastest speed it could reach on a motion it keeps when handing off
const float CHAIN_LATERAL_CARRY = 0.5;
const float CHAIN_ANGULAR_CARRY = 0.25;
// lemlib's moveToPoint stops steering once it's this close (inches) to the target
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
    // waits for the running motion to end so the pose is where the robot actually is. lemlib
    // waits for it anyway before starting the next motion, but the chain values would've been
    // worked out from where the robot was when the last motion started
    void waitForMotion(){
        while(isInMotion()) pros::delay(5);
    }

    // drive top speeds at full power, from the drivetrain settings
    float lateralTopSpeed(){
        return drivetrain.rpm / 60 * M_PI * drivetrain.wheelDiameter;  // in/s
    }
    float angularTopSpeed(){
        return lateralTopSpeed() * 2 / drivetrain.trackWidth * 180 / M_PI;  // deg/s
    }

    /**
     * @brief works out how fast a chained motion should still be going when it hands off, and
     * how early it should exit so it coasts the rest of the way to its target
     *
     * The speed it carries is a fraction of the fastest it could reach on a motion that long
     * (sqrt(2 * accel * distance)), and it exits as far early as it takes to slow down from that
     * speed (speed^2 / (2 * accel)), so short motions carry less speed and exit less early.
     */
    void chainValues(float distance, float topSpeed, float maxSpeed, float accel, float carry,
                     float& minSpeed, float& earlyExitRange){
        float reachable = std::min(std::sqrt(2 * accel * distance), topSpeed * maxSpeed / 127);
        float carrySpeed = reachable * carry;
        minSpeed = carrySpeed / topSpeed * 127;
        earlyExitRange = std::min(carrySpeed * carrySpeed / (2 * accel), distance / 2);
    }

    public:
    using lemlib::Chassis::Chassis;

    void chainToPoint(float x, float y, int timeout, ChainToPointParams params = {}, bool async = true){
        waitForMotion();
        lemlib::Pose start = getPose();
        float distance = std::hypot(x - start.x, y - start.y);

        // lemlib stops steering once it's within 7.5in of the target, so a short move to a point that
        // isn't straight ahead/behind would drive right past it. turn to face the point first (for
        // really short moves, whichever end of the robot is closer, since lemlib drives either way then)
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
            chainValues(distance, lateralTopSpeed(), std::abs(params.maxSpeed), CHAIN_LATERAL_ACCEL,
                        CHAIN_LATERAL_CARRY, minSpeed, earlyExitRange);
        }

        moveToPoint(x, y, timeout,
                    {.forwards = params.forwards,
                     .maxSpeed = (float) params.maxSpeed,
                     .minSpeed = minSpeed,
                     .earlyExitRange = earlyExitRange},
                    async);
    }

    void chainToPose(float x, float y, float theta, int timeout, ChainToPoseParams params = {}, bool async = true){
        waitForMotion();
        lemlib::Pose start = getPose();
        float distance = std::hypot(x - start.x, y - start.y);

        float minSpeed = 0, earlyExitRange = 0;
        if(params.chain){
            chainValues(distance, lateralTopSpeed(), std::abs(params.maxSpeed), CHAIN_LATERAL_ACCEL,
                        CHAIN_LATERAL_CARRY, minSpeed, earlyExitRange);
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

    void chainToHeading(float theta, int timeout, ChainToHeadingParams params = {}, bool async = true){
        waitForMotion();
        float error = lemlib::angleError(theta, getPose().theta, false, params.direction);
        float distance = std::abs(error);

        float minSpeed = 0, earlyExitRange = 0;
        if(params.chain){
            chainValues(distance, angularTopSpeed(), std::abs(params.maxSpeed), CHAIN_ANGULAR_ACCEL,
                        CHAIN_ANGULAR_CARRY, minSpeed, earlyExitRange);
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