#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"
#include "robot/drivetrain.h"
#include "robot/odometry.h"
#include "robot/chain.h"
#include <atomic>
#include <functional>

using lemlib::angleError;
using lemlib::degToRad;
using lemlib::radToDeg;

static float clampf(float value, float low, float high){
    return std::max(low, std::min(value, high));
}

static float sign(float value){
    return value < 0 ? -1 : 1;
}

// limit how fast the output speeds up (never limits slowing down)
static float slewUp(float out, float prev, float slew){
    if(slew <= 0) return out;
    if(out * prev < 0) prev = 0;
    if(std::abs(out) > std::abs(prev)) out = prev + clampf(out - prev, -slew, slew);
    return out;
}

float CustomChassis::lateralTopSpeed(){
    return drivetrain.rpm / 60 * M_PI * drivetrain.wheelDiameter;
}

float CustomChassis::angularTopSpeed(){
    return lateralTopSpeed() * 2 / drivetrain.trackWidth * 180 / M_PI;
}

/**
 * @brief start measuring the robot's speed
 *
 * Measures forward and turning speed from odometry over the last 30ms, every 10ms.
 */
void CustomChassis::startTracker(){
    if(trackerStarted) return;
    trackerStarted = true;

    pros::Task tracker([this](){
        const int SAMPLES = 4;   // 4 samples = 30ms window
        lemlib::Pose poses[SAMPLES] = {lemlib::Pose(0, 0), lemlib::Pose(0, 0), lemlib::Pose(0, 0), lemlib::Pose(0, 0)};
        uint32_t times[SAMPLES] = {0, 0, 0, 0};
        int count = 0, next = 0;

        while(true){
            lemlib::Pose pose = getPose();
            uint32_t now = pros::millis();

            if(count == SAMPLES){
                lemlib::Pose old = poses[next];   // oldest sample
                float dt = (now - times[next]) / 1000.0f;
                if(dt > 0){
                    float heading = degToRad(pose.theta);
                    float forward = ((pose.x - old.x) * std::sin(heading) + (pose.y - old.y) * std::cos(heading)) / dt;
                    float turn = angleError(pose.theta, old.theta, false) / dt;

                    // a jump this big is a setPose(), not real movement
                    if(std::abs(forward) > 3 * lateralTopSpeed() || std::abs(turn) > 3 * angularTopSpeed()){
                        count = 0;
                    }else{
                        trackedForward = forward;
                        trackedTurn = turn;
                    }
                }
            }

            poses[next] = pose;
            times[next] = now;
            next = (next + 1) % SAMPLES;
            if(count < SAMPLES) count++;

            pros::delay(10);
        }
    }, "chain speed");
}

/**
 * @brief drive to a point or pose (runs inside a motion)
 *
 * PID drive with steering like LemLib, except the speed is capped by a braking curve
 * (speed = sqrt(2 * DECEL * distance left)) so the robot can always stop at the target.
 * When another motion is queued, it follows the curve and hands off once it's within
 * CHAIN_LATERAL_EXIT and slow enough to stop in the distance left.
 */
void CustomChassis::driveLoop(float x, float y, std::optional<float> theta, bool forwards, float maxSpeed,
                              float lead, float horizontalDrift, bool chain, uint32_t deadline){
    const float top = lateralTopSpeed();
    const float decel = std::max(CHAIN_LATERAL_DECEL, 1.0f);
    auto power = [top](float speed){ return speed / top * 127; };

    lemlib::Pose start = getPose(true, true);
    lemlib::Pose goal(x, y, 0);
    if(theta){
        goal.theta = M_PI_2 - degToRad(*theta);
        if(!forwards) goal.theta = std::fmod(goal.theta + M_PI, 2 * M_PI);
    }
    if(horizontalDrift <= 0) horizontalDrift = drivetrain.horizontalDrift;

    // direction the robot finishes the motion in (used to check how far is left)
    float pathLength = start.distance(goal);
    float pathX = 0, pathY = 0;
    if(theta){
        pathX = std::cos(goal.theta);
        pathY = std::sin(goal.theta);
    }else if(pathLength > 0.01){
        pathX = (goal.x - start.x) / pathLength;
        pathY = (goal.y - start.y) / pathLength;
    }
    float exitRange = std::min(CHAIN_LATERAL_EXIT, pathLength / 2);

    lateralPID.reset();
    angularPID.reset();
    lateralSmallExit.reset();
    lateralLargeExit.reset();

    bool close = false;
    bool first = true;
    bool braking = false;     // still braking off speed left over from the last motion
    float brakeFrom = 0;
    uint32_t brakeStart = pros::millis();
    float prevOut = 0;
    lemlib::Pose lastPose = start;
    ChainExit result;

    while(pros::millis() < deadline && motionRunning){
        lemlib::Pose pose = getPose(true, true);
        distTraveled += pose.distance(lastPose);
        lastPose = pose;

        float distGoal = pose.distance(goal);
        float remaining = (goal.x - pose.x) * pathX + (goal.y - pose.y) * pathY;
        if(distGoal < CHAIN_SETTLE_RANGE) close = true;

        // where to aim: the boomerang carrot for poses, the point itself otherwise
        lemlib::Pose aim = goal;
        if(theta && !close) aim = goal - lemlib::Pose(std::cos(goal.theta), std::sin(goal.theta)) * lead * distGoal;

        float facing = forwards ? pose.theta : pose.theta + M_PI;
        float angularError = (theta && close) ? angleError(facing, goal.theta) : angleError(facing, pose.angle(aim));
        float lateralError = pose.distance(aim);
        if(theta && !close) lateralError *= sign(std::cos(angleError(pose.theta, pose.angle(aim))));
        else lateralError *= std::cos(angleError(pose.theta, pose.angle(aim)));

        float dir = sign(lateralError);                 // + = drive forwards to get there
        float toward = trackedForward * dir;            // in/s toward the target
        float brakeDist = (theta && !close) ? distGoal : std::abs(lateralError);

        if(first){
            first = false;
            // moving away from the target (drive then reverse): brake that off along the curve first
            if(toward < -2){
                braking = true;
                brakeFrom = -toward;
                brakeStart = pros::millis();
            }else{
                prevOut = clampf(power(trackedForward), -maxSpeed, maxSpeed);
            }
        }

        float pidOut = lateralPID.update(lateralError);
        float angularOut = angularPID.update(radToDeg(angularError));
        if(close && !theta) angularOut = 0;

        // fastest the robot can go and still stop at the target
        float curveSpeed = std::sqrt(2 * decel * brakeDist);
        float curveOut = power(curveSpeed) + CHAIN_SPEED_KV * power(curveSpeed - toward);
        bool handoff = chain && motionQueued;

        float drive;   // power toward the target
        if(braking){
            float awaySpeed = std::max(0.0f, brakeFrom - decel * (pros::millis() - brakeStart) / 1000.0f);
            drive = power(-awaySpeed) + CHAIN_SPEED_KV * power(-awaySpeed - toward);
            angularOut = 0;
            if(awaySpeed <= 0) braking = false;
        }else if(handoff){
            drive = std::min(curveOut, maxSpeed);
        }else{
            drive = std::min({std::max(pidOut * dir, close ? -maxSpeed : 0.0f), curveOut, maxSpeed});
        }
        drive = std::max(drive, -127.0f);

        // hand off to the next motion once the robot can stop in the distance left
        if(handoff && !braking && (close || !theta)){
            float stopDist = toward > 0 ? toward * toward / (2 * decel) : 0;
            if((remaining <= exitRange && stopDist <= remaining + 0.25) || remaining <= 0){
                result = {true, remaining, toward};
                break;
            }
        }

        // settle like LemLib when nothing is queued after this
        lateralSmallExit.update(lateralError);
        lateralLargeExit.update(lateralError);
        if(!handoff && close && (lateralSmallExit.getExit() || lateralLargeExit.getExit())){
            result = {false, remaining, toward};
            break;
        }

        float lateralOut = dir * drive;
        if(!close && !braking) lateralOut = slewUp(lateralOut, prevOut, lateralSettings.slew);
        prevOut = lateralOut;

        angularOut = clampf(angularOut, -maxSpeed, maxSpeed);
        float limit = std::min(127.0f, std::max(maxSpeed, std::abs(lateralOut)));

        if(theta && !close){
            // don't go faster than the wheels can grip around the curve
            float curvature = std::abs(lemlib::getCurvature(pose, aim));
            if(curvature > 0){
                float slip = std::sqrt(horizontalDrift / curvature * 9.8f);
                lateralOut = clampf(lateralOut, -slip, slip);
            }
            // turning comes first
            float overturn = std::abs(angularOut) + std::abs(lateralOut) - limit;
            if(overturn > 0) lateralOut -= sign(lateralOut) * overturn;
        }

        float left = lateralOut + angularOut;
        float right = lateralOut - angularOut;
        float ratio = std::max(std::abs(left), std::abs(right)) / limit;
        if(ratio > 1){
            left /= ratio;
            right /= ratio;
        }
        drivetrain.leftMotors->move(left);
        drivetrain.rightMotors->move(right);

        pros::delay(10);
    }

    // a handoff keeps the motors going so the next motion picks up the speed
    if(!result.handedOff){
        drivetrain.leftMotors->move(0);
        drivetrain.rightMotors->move(0);
    }
    lastExit = result;

    if(CHAIN_DEBUG){
        if(result.handedOff){
            printf("[chain] to (%.1f, %.1f): handed off %.2fin early at %.1fin/s (stops in %.2fin)\n",
                   x, y, result.remaining, result.speed, result.speed > 0 ? result.speed * result.speed / (2 * decel) : 0.0f);
        }else{
            printf("[chain] to (%.1f, %.1f): stopped %.2fin from the target\n", x, y, result.remaining);
        }
    }
}

/**
 * @brief turn to a heading (runs inside a motion)
 *
 * PID turn like LemLib, except the speed is capped by a braking curve
 * (speed = sqrt(2 * DECEL * angle left)). When another motion is queued (or forceHandoff),
 * it follows the curve and hands off once it's within CHAIN_ANGULAR_EXIT and slow enough
 * to stop in the angle left. Also brakes off forward speed left over from a chained drive.
 */
void CustomChassis::turnLoop(float theta, lemlib::AngularDirection direction, float maxSpeed, bool chain,
                             bool forceHandoff, uint32_t deadline){
    const float topTurn = angularTopSpeed();
    const float topDrive = lateralTopSpeed();
    const float decel = std::max(CHAIN_ANGULAR_DECEL, 1.0f);
    const float driveDecel = std::max(CHAIN_LATERAL_DECEL, 1.0f);
    auto power = [topTurn](float speed){ return speed / topTurn * 127; };

    float lastTheta = getPose().theta;
    float total = angleError(theta, lastTheta, false, direction);
    float turned = 0;
    float exitRange = std::min(CHAIN_ANGULAR_EXIT, std::abs(total) / 2);
    float startDir = sign(total);

    angularPID.reset();
    angularSmallExit.reset();
    angularLargeExit.reset();

    // speed left over from the last motion
    float driftFrom = trackedForward;
    float spinFrom = trackedTurn;
    bool braking = spinFrom * startDir < -5;   // spinning the wrong way: brake that off first
    float prevOut = braking ? 0 : clampf(power(spinFrom), -maxSpeed, maxSpeed);
    uint32_t startTime = pros::millis();
    ChainExit result;

    while(pros::millis() < deadline && motionRunning){
        float heading = getPose().theta;
        turned += angleError(heading, lastTheta, false);
        lastTheta = heading;
        distTraveled = std::abs(turned);

        float remaining = total - turned;
        float dir = sign(remaining);
        float toward = trackedTurn * dir;   // deg/s toward the target
        float elapsed = (pros::millis() - startTime) / 1000.0f;

        float pidOut = angularPID.update(remaining);
        float curveSpeed = std::sqrt(2 * decel * std::abs(remaining));
        float curveOut = power(curveSpeed) + CHAIN_SPEED_KV * power(curveSpeed - toward);
        bool handoff = chain && (forceHandoff || motionQueued);

        float drive;   // power toward the target
        if(braking){
            float awaySpeed = std::max(0.0f, std::abs(spinFrom) - decel * elapsed);
            drive = power(-awaySpeed) + CHAIN_SPEED_KV * power(-awaySpeed - toward);
            if(awaySpeed <= 0) braking = false;
        }else if(handoff){
            drive = std::min(curveOut, maxSpeed);
        }else{
            drive = std::min({pidOut * dir, curveOut, maxSpeed});
        }
        drive = std::max(drive, -127.0f);

        // hand off to the next motion once the robot can stop in the angle left
        if(handoff && !braking){
            float stopAngle = toward > 0 ? toward * toward / (2 * decel) : 0;
            if((std::abs(remaining) <= exitRange && stopAngle <= std::abs(remaining) + 0.5f) || remaining * startDir <= 0){
                result = {true, std::abs(remaining), toward};
                break;
            }
        }

        angularSmallExit.update(remaining);
        angularLargeExit.update(remaining);
        if(!handoff && (angularSmallExit.getExit() || angularLargeExit.getExit())){
            result = {false, std::abs(remaining), toward};
            break;
        }

        float turnOut = dir * drive;
        if(std::abs(remaining) > 20 && !braking) turnOut = slewUp(turnOut, prevOut, angularSettings.slew);
        prevOut = turnOut;

        // brake off forward/backward speed from a chained drive along the drive braking curve
        float driftSpeed = sign(driftFrom) * std::max(0.0f, std::abs(driftFrom) - driveDecel * elapsed);
        float driveOut = (driftSpeed + CHAIN_SPEED_KV * (driftSpeed - trackedForward)) / topDrive * 127;
        driveOut = clampf(driveOut, -127, 127);

        float left = driveOut + turnOut;
        float right = driveOut - turnOut;
        float ratio = std::max(std::abs(left), std::abs(right)) / 127;
        if(ratio > 1){
            left /= ratio;
            right /= ratio;
        }
        drivetrain.leftMotors->move(left);
        drivetrain.rightMotors->move(right);

        pros::delay(10);
    }

    if(!result.handedOff){
        drivetrain.leftMotors->move(0);
        drivetrain.rightMotors->move(0);
    }
    lastExit = result;

    if(CHAIN_DEBUG){
        if(result.handedOff){
            printf("[chain] to %.1f deg: handed off %.1fdeg early at %.0fdeg/s (stops in %.1fdeg)\n",
                   theta, result.remaining, result.speed, result.speed > 0 ? result.speed * result.speed / (2 * decel) : 0.0f);
        }else{
            printf("[chain] to %.1f deg: stopped %.1fdeg from the target\n", theta, result.remaining);
        }
    }
}

void CustomChassis::chainToPoint(float x, float y, int timeout, ChainToPointParams params, bool async){
    requestMotionStart();
    if(!motionRunning) return;
    if(async){
        pros::Task task([this, x, y, timeout, params](){ chainToPoint(x, y, timeout, params, false); });
        endMotion();
        pros::delay(10);
        return;
    }

    startTracker();
    uint32_t deadline = pros::millis() + timeout;
    float maxSpeed = std::abs(params.maxSpeed);
    distTraveled = 0;

    // short moves don't steer, so face the point first
    lemlib::Pose start = getPose();
    float distance = std::hypot(x - start.x, y - start.y);
    if(distance < 2 * CHAIN_SETTLE_RANGE && distance > 0.5){
        float bearing = radToDeg(std::atan2(x - start.x, y - start.y));
        if(distance < CHAIN_SETTLE_RANGE){
            params.forwards = std::abs(angleError(bearing, start.theta, false)) <= 90;
        }
        float facing = params.forwards ? bearing : bearing + 180;

        if(std::abs(angleError(facing, start.theta, false)) > CHAIN_FACE_TOLERANCE){
            turnLoop(facing, lemlib::AngularDirection::AUTO, maxSpeed, true, true, deadline);
            distTraveled = 0;
        }
    }

    if(motionRunning){
        driveLoop(x, y, std::nullopt, params.forwards, maxSpeed, 0, 0, params.chain, deadline);
    }

    distTraveled = -1;
    endMotion();
}

void CustomChassis::chainToPose(float x, float y, float theta, int timeout, ChainToPoseParams params, bool async){
    requestMotionStart();
    if(!motionRunning) return;
    if(async){
        pros::Task task([this, x, y, theta, timeout, params](){ chainToPose(x, y, theta, timeout, params, false); });
        endMotion();
        pros::delay(10);
        return;
    }

    startTracker();
    distTraveled = 0;
    driveLoop(x, y, theta, params.forwards, std::abs(params.maxSpeed), params.lead, params.horizontalDrift,
              params.chain, pros::millis() + timeout);

    distTraveled = -1;
    endMotion();
}

void CustomChassis::chainToHeading(float theta, int timeout, ChainToHeadingParams params, bool async){
    requestMotionStart();
    if(!motionRunning) return;
    if(async){
        pros::Task task([this, theta, timeout, params](){ chainToHeading(theta, timeout, params, false); });
        endMotion();
        pros::delay(10);
        return;
    }

    startTracker();
    distTraveled = 0;
    turnLoop(theta, params.direction, std::abs(params.maxSpeed), params.chain, false, pros::millis() + timeout);

    distTraveled = -1;
    endMotion();
}

/* ===================================== TUNER ===================================== */

static std::atomic<bool> chain_tune_aborted = false;

/**
 * @brief check if B was pressed to stop the tuner
 */
static bool chain_tune_stop(){
    if(!chain_tune_aborted && controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
        chain_tune_aborted = true;
        chassis.cancelAllMotions();
        left_mg.move(0);
        right_mg.move(0);
        printf("[chain tuner] stopped with B\n");
        pros::lcd::print(1, "chain tuner: stopped");
    }
    return chain_tune_aborted;
}

/**
 * @brief drive full speed, slam into full reverse, and measure how hard the robot stopped
 * @param[in] dir 1 = forwards, -1 = backwards
 * @return braking in in/s^2 (0 if stopped)
 */
static float measure_lateral_brake(int dir){
    chassis.setPose(0, 0, 0);
    pros::delay(150);

    uint32_t start = pros::millis();
    left_mg.move(127 * dir);
    right_mg.move(127 * dir);
    while(chassis.getPose().y * dir < 30 && pros::millis() - start < 1500 && !chain_tune_stop()) pros::delay(10);

    float speed = std::abs(chassis.forwardSpeed());
    float from = chassis.getPose().y * dir;
    float furthest = from;

    start = pros::millis();
    left_mg.move(-127 * dir);
    right_mg.move(-127 * dir);
    while(chassis.forwardSpeed() * dir > 0 && pros::millis() - start < 1000 && !chain_tune_stop()){
        furthest = std::max(furthest, (float) chassis.getPose().y * dir);
        pros::delay(10);
    }
    left_mg.move(0);
    right_mg.move(0);
    pros::delay(400);

    float stopDist = std::max(furthest - from, 0.25f);
    float decel = speed * speed / (2 * stopDist);
    printf("[chain tuner] drive brake (%s): %.1fin/s stopped in %.2fin -> %.0fin/s^2\n",
           dir > 0 ? "forwards" : "backwards", speed, stopDist, decel);
    return chain_tune_aborted ? 0 : decel;
}

/**
 * @brief spin full speed, slam into full reverse, and measure how hard the robot stopped
 * @param[in] dir 1 = clockwise, -1 = counterclockwise
 * @return braking in deg/s^2 (0 if stopped)
 */
static float measure_angular_brake(int dir){
    chassis.setPose(0, 0, 0);
    pros::delay(150);

    // add up the turn in small steps so it works past 360
    float turned = 0, last = chassis.getPose().theta;
    auto update = [&](){
        float heading = chassis.getPose().theta;
        turned += angleError(heading, last, false) * dir;
        last = heading;
    };

    uint32_t start = pros::millis();
    left_mg.move(127 * dir);
    right_mg.move(-127 * dir);
    while(turned < 270 && pros::millis() - start < 1200 && !chain_tune_stop()){
        pros::delay(10);
        update();
    }

    float speed = std::abs(chassis.turnSpeed());
    float from = turned;
    float furthest = from;

    start = pros::millis();
    left_mg.move(-127 * dir);
    right_mg.move(127 * dir);
    while(chassis.turnSpeed() * dir > 0 && pros::millis() - start < 1000 && !chain_tune_stop()){
        pros::delay(10);
        update();
        furthest = std::max(furthest, turned);
    }
    left_mg.move(0);
    right_mg.move(0);
    pros::delay(400);

    float stopAngle = std::max(furthest - from, 1.0f);
    float decel = speed * speed / (2 * stopAngle);
    printf("[chain tuner] turn brake (%s): %.0fdeg/s stopped in %.1fdeg -> %.0fdeg/s^2\n",
           dir > 0 ? "clockwise" : "counterclockwise", speed, stopAngle, decel);
    return chain_tune_aborted ? 0 : decel;
}

/**
 * @brief run some motions and record the furthest the robot got along y (or theta)
 * @param[in] motions starts the motions (the last one should block, or call waitUntilDone)
 * @param[in] use_theta true = record heading instead of y
 * @return the furthest y (or heading) reached
 */
static float furthest_during(std::function<void()> motions, bool use_theta){
    static std::atomic<bool> watching;
    static float furthest;
    watching = true;
    furthest = -1e9;

    pros::Task watcher([use_theta](){
        while(watching){
            lemlib::Pose pose = chassis.getPose();
            furthest = std::max(furthest, use_theta ? angleError(pose.theta, 0, false) : pose.y);
            chain_tune_stop();
            pros::delay(10);
        }
    });

    motions();
    chassis.waitUntilDone();
    pros::delay(300);   // anything coasting past the target still counts
    watching = false;
    pros::delay(30);
    return furthest;
}

/**
 * @brief run the chain tests with the current DECEL values
 * @param[out] drive_over worst overshoot of a drive (in)
 * @param[out] turn_over worst overshoot of a turn (deg)
 */
static void chain_tests(float& drive_over, float& turn_over){
    const float D = 30;
    drive_over = -1e9;
    turn_over = -1e9;

    // drive then reverse
    if(!chain_tune_aborted){
        chassis.setPose(0, 0, 0);
        float y = furthest_during([D](){
            chassis.chainToPoint(0, D, 2500);
            chassis.chainToPoint(0, 0, 2500, {.forwards = false});
        }, false);
        drive_over = std::max(drive_over, y - D);
        printf("[chain tuner]   drive -> reverse: %+.2fin past the target\n", y - D);
        pros::delay(300);
    }

    // drive then turn
    if(!chain_tune_aborted){
        chassis.setPose(0, 0, 0);
        float y = furthest_during([D](){
            chassis.chainToPoint(0, D, 2500);
            chassis.chainToHeading(90, 1500);
        }, false);
        drive_over = std::max(drive_over, y - D);
        printf("[chain tuner]   drive -> turn:    %+.2fin past the target\n", y - D);
        if(!chain_tune_aborted){
            chassis.turnToHeading(0, 1500, {}, false);
            chassis.moveToPoint(0, 0, 2500, {.forwards = false}, false);
        }
        pros::delay(300);
    }

    // drive then stop (nothing chained after it)
    if(!chain_tune_aborted){
        chassis.setPose(0, 0, 0);
        float y = furthest_during([D](){
            chassis.chainToPoint(0, D, 2500, {}, false);
        }, false);
        drive_over = std::max(drive_over, y - D);
        printf("[chain tuner]   drive -> stop:    %+.2fin past the target\n", y - D);
        if(!chain_tune_aborted) chassis.moveToPoint(0, 0, 2500, {.forwards = false}, false);
        pros::delay(300);
    }

    // turn then turn back
    if(!chain_tune_aborted){
        chassis.setPose(0, 0, 0);
        float theta = furthest_during([](){
            chassis.chainToHeading(90, 1500);
            chassis.chainToHeading(0, 1500);
        }, true);
        turn_over = std::max(turn_over, theta - 90);
        printf("[chain tuner]   turn -> turn back: %+.1fdeg past the target\n", theta - 90);
        pros::delay(300);
    }

    // turn then stop
    if(!chain_tune_aborted){
        chassis.setPose(0, 0, 0);
        float theta = furthest_during([](){
            chassis.chainToHeading(90, 1500, {}, false);
        }, true);
        turn_over = std::max(turn_over, theta - 90);
        printf("[chain tuner]   turn -> stop:     %+.1fdeg past the target\n", theta - 90);
        if(!chain_tune_aborted) chassis.turnToHeading(0, 1500, {}, false);
        pros::delay(300);
    }
}

/**
 * @brief tune CHAIN_LATERAL_DECEL and CHAIN_ANGULAR_DECEL
 *
 * Needs ~4ft of open space in front of the robot. Keep the lift down.
 * (1) Measures how hard the robot can brake driving and turning, and sets DECEL to 75% of it.
 * (2) Runs drive->reverse, drive->turn, drive->stop, turn->turn back, and turn->stop. If anything
 * goes more than 0.5in / 2deg past its target, lowers that DECEL and tries again.
 * Prints the values to paste into robot/chain.h. Press B at any time to stop.
 */
void tune_chain(){
    chain_tune_aborted = false;
    chassis.startTracker();

    printf("\n[chain tuner] press A to start (B = stop). needs ~4ft clear in front, lift down\n");
    pros::lcd::print(0, "chain tuner: press A to start");
    pros::lcd::print(1, "needs ~4ft clear in front, lift down. B = stop");
    while(!controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)){
        if(chain_tune_stop()) return;
        pros::delay(20);
    }
    while(controller.get_digital(pros::E_CONTROLLER_DIGITAL_A)) pros::delay(20);
    pros::lcd::print(0, "chain tuner: running (B = stop)");
    pros::delay(500);

    // 1: how hard can it brake
    pros::lcd::print(1, "measuring braking...");
    float drive_brake = std::min(measure_lateral_brake(1), measure_lateral_brake(-1));
    float turn_brake = std::min(measure_angular_brake(1), measure_angular_brake(-1));
    if(chain_tune_aborted) return;

    if(drive_brake > 0) CHAIN_LATERAL_DECEL = 0.75f * drive_brake;
    if(turn_brake > 0) CHAIN_ANGULAR_DECEL = 0.75f * turn_brake;

    // 2: check for overshoot, back off until there is none
    bool drive_ok = false, turn_ok = false;
    for(int round = 1; round <= 4 && !chain_tune_aborted; round++){
        printf("[chain tuner] round %d: CHAIN_LATERAL_DECEL %.0f, CHAIN_ANGULAR_DECEL %.0f\n",
               round, CHAIN_LATERAL_DECEL, CHAIN_ANGULAR_DECEL);
        pros::lcd::print(1, "round %d: drive %.0f  turn %.0f", round, CHAIN_LATERAL_DECEL, CHAIN_ANGULAR_DECEL);

        float drive_over, turn_over;
        chain_tests(drive_over, turn_over);
        if(chain_tune_aborted) break;

        drive_ok = drive_over <= 0.5f;
        turn_ok = turn_over <= 2.0f;
        pros::lcd::print(2, "overshoot: drive %.2fin  turn %.1fdeg", drive_over, turn_over);
        if(drive_ok && turn_ok) break;

        if(!drive_ok) CHAIN_LATERAL_DECEL *= 0.85f;
        if(!turn_ok) CHAIN_ANGULAR_DECEL *= 0.85f;
    }

    if(chain_tune_aborted) return;

    printf("\n// chain tuner %s - paste these into include/robot/chain.h\n",
           drive_ok && turn_ok ? "done" : "ran out of rounds, still some overshoot");
    printf("inline float CHAIN_LATERAL_DECEL = %.0f;\n", CHAIN_LATERAL_DECEL);
    printf("inline float CHAIN_ANGULAR_DECEL = %.0f;\n\n", CHAIN_ANGULAR_DECEL);
    fflush(stdout);

    pros::lcd::print(0, "chain tuner: %s", drive_ok && turn_ok ? "done" : "done (still overshoots)");
    pros::lcd::print(3, "CHAIN_LATERAL_DECEL = %.0f", CHAIN_LATERAL_DECEL);
    pros::lcd::print(4, "CHAIN_ANGULAR_DECEL = %.0f", CHAIN_ANGULAR_DECEL);
}
