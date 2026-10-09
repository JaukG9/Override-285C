#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include "robot/drivetrain.h"
#include "robot/odometry.h"

extern pros::Imu imu;

struct ChainParams{
    float earlyExitRange;
    float minSpeed;
};
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
    std::optional<lemlib::Pose> lastTarget;
    std::optional<float> lastHeading;

    static float wrap180(float angle){
        angle = std::fmod(angle, 360.0f);
        if(angle > 180) angle -= 360;
        if(angle < -180) angle += 360;
        return angle;
    }

    public:
    using lemlib::Chassis::Chassis;

    // Call this if you run a plain moveToPoint/turnToHeading/etc. between chained calls
    void resetChain(){
        lastTarget = std::nullopt;
        lastHeading = std::nullopt;
    }

    void chainToPoint(float x, float y, int timeout, ChainToPointParams params = {}, bool async = true){
        lemlib::Pose start = lastTarget.value_or(getPose());

        float dx = x - start.x;
        float dy = y - start.y;
        float distance = std::hypot(dx, dy);
        float actualMax = std::min((float) std::abs(params.maxSpeed), std::sqrt(1000 * distance));

        float minSpeed = params.chain ? actualMax / 2 : 0;
        float earlyExitRange = params.chain ? distance * 0.2f * actualMax / 127 : 0;

        if(params.chain){
            lastTarget = lemlib::Pose(x, y, 0);
            // heading the robot ends up facing (0 deg = +y, clockwise positive)
            float travel = std::atan2(dx, dy) * 180.0f / M_PI;
            lastHeading = params.forwards ? travel : travel + 180.0f;
        } else {
            resetChain();
        }

        moveToPoint(x, y, timeout,
                    {.forwards = params.forwards,
                     .maxSpeed = params.maxSpeed,
                     .minSpeed = (int) minSpeed,
                     .earlyExitRange = earlyExitRange},
                    async);
    }

    void chainToPose(float x, float y, float theta, int timeout, ChainToPoseParams params = {}, bool async = true){
        lemlib::Pose start = lastTarget.value_or(getPose());

        float distance = std::hypot(x - start.x, y - start.y);
        float actualMax = std::min((float) std::abs(params.maxSpeed), std::sqrt(1000 * distance));

        float minSpeed = params.chain ? actualMax / 2 : 0;
        float earlyExitRange = params.chain ? distance * 0.2f * actualMax / 127 : 0;

        if(params.chain){
            lastTarget = lemlib::Pose(x, y, theta);
            lastHeading = theta;
        } else {
            resetChain();
        }

        moveToPose(x, y, theta, timeout,
                   {.forwards = params.forwards,
                    .horizontalDrift = params.horizontalDrift,
                    .lead = params.lead,
                    .maxSpeed = params.maxSpeed,
                    .minSpeed = (int) minSpeed,
                    .earlyExitRange = earlyExitRange},
                   async);
    }

    void chainToHeading(float theta, int timeout, ChainToHeadingParams params = {}, bool async = true){
        float startHeading = lastHeading.value_or(getPose().theta);

        // wrapped so a 355 -> 5 turn counts as 10 degrees, not 350
        float distance = std::abs(wrap180(theta - startHeading));
        float actualMax = std::min((float) std::abs(params.maxSpeed), std::sqrt(500 * distance));

        float minSpeed = params.chain ? actualMax / 2 : 0;
        float earlyExitRange = params.chain ? distance * 0.1f * actualMax / 127 : 0;

        if(params.chain){
            lastHeading = theta;
        } else {
            resetChain();
        }

        turnToHeading(theta, timeout,
                      {.direction = params.direction,
                       .maxSpeed = params.maxSpeed,
                       .minSpeed = (int) minSpeed,
                       .earlyExitRange = earlyExitRange},
                      async);
    }
};

extern CustomChassis chassis;

extern double target;
extern bool do_lift_PID;
void lift_PID(void*);