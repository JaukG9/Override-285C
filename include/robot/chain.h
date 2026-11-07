#pragma once

#include "main.h"
#include "lemlib/api.hpp"
#include <optional>

/* =====================================================================================
 *  MOTION CHAINING - TUNING (everything you need to change is in this block)
 *
 *  Easiest way: set RUN_CHAIN_TUNER to true, upload, run the program NOT plugged into a
 *  field / comp switch, and press A when the brain says to. The robot measures how hard
 *  it can brake, tests itself for overshoot, and prints two lines in the terminal.
 *  Paste them over the two DECEL lines below, then set RUN_CHAIN_TUNER back to false.
 *
 *  Tuning by hand:
 *    - robot goes past the target       -> lower that DECEL by ~15%
 *    - robot stops early / creeps in    -> raise that DECEL by ~15%
 *    - want more speed between motions  -> raise that EXIT (it carries sqrt(2 * DECEL * EXIT))
 * ===================================================================================== */

// true = run tune_chain() at the start of driver control (never runs when plugged into a field)
inline constexpr bool RUN_CHAIN_TUNER = false;

// how hard the robot brakes coming into a target, in in/s^2
inline float CHAIN_LATERAL_DECEL = 150;
// same thing for turns, in deg/s^2
inline float CHAIN_ANGULAR_DECEL = 1200;

// how far before the target (in) a chained drive hands off to the next motion
inline float CHAIN_LATERAL_EXIT = 3;
// same thing for turns, in degrees
inline float CHAIN_ANGULAR_EXIT = 5;

// true = print a line to the terminal at the end of every chainTo* motion
inline bool CHAIN_DEBUG = false;

/* ---- usually leave these alone ---- */
// how hard the robot corrects when it's faster/slower than the braking curve
inline constexpr float CHAIN_SPEED_KV = 1.5;
// within this distance (in) a drive stops steering (same as LemLib)
inline constexpr float CHAIN_SETTLE_RANGE = 7.5;
// how far off (deg) a close point can be before chainToPoint turns to face it first
inline constexpr float CHAIN_FACE_TOLERANCE = 10;
/* ===================================================================================== */

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

/**
 * How chaining works:
 *  - Every drive/turn brakes along a curve set by DECEL, so it always arrives at a speed it
 *    can stop from.
 *  - A motion only hands off early if another motion is already waiting after it. The last
 *    motion before a waitUntilDone(), delay, or the end of the auton always stops normally,
 *    so chain = true is safe to leave on everywhere.
 *  - The handoff happens when the robot's measured speed can stop in the distance left, so
 *    the next motion lands the robot on the old target instead of past it.
 *  - The next chainTo* motion brakes off any leftover speed along the same curve (drive then
 *    reverse, or drive then turn) instead of slamming or coasting.
 */
class CustomChassis : public lemlib::Chassis{
    public:
    using lemlib::Chassis::Chassis;

    /**
     * @brief drive to a point, chaining into the next motion if one is queued
     */
    void chainToPoint(float x, float y, int timeout, ChainToPointParams params = {}, bool async = true);

    /**
     * @brief drive to a pose (boomerang), chaining into the next motion if one is queued
     */
    void chainToPose(float x, float y, float theta, int timeout, ChainToPoseParams params = {}, bool async = true);

    /**
     * @brief turn to a heading, chaining into the next motion if one is queued
     */
    void chainToHeading(float theta, int timeout, ChainToHeadingParams params = {}, bool async = true);

    // robot speed measured from odometry (updated every 10ms)
    float forwardSpeed(){ return trackedForward; }   // in/s, + = forwards
    float turnSpeed(){ return trackedTurn; }         // deg/s, + = clockwise
    void startTracker();

    // how the last chainTo* motion ended
    struct ChainExit{
        bool handedOff = false;   // true = exited early into the next motion
        float remaining = 0;      // in or deg left when it exited
        float speed = 0;          // in/s or deg/s toward the target when it exited
    };
    ChainExit lastExit;

    private:
    float lateralTopSpeed();
    float angularTopSpeed();
    void driveLoop(float x, float y, std::optional<float> theta, bool forwards, float maxSpeed,
                   float lead, float horizontalDrift, bool chain, uint32_t deadline);
    void turnLoop(float theta, lemlib::AngularDirection direction, float maxSpeed, bool chain,
                  bool forceHandoff, uint32_t deadline);

    volatile float trackedForward = 0;
    volatile float trackedTurn = 0;
    bool trackerStarted = false;
};

void tune_chain();
