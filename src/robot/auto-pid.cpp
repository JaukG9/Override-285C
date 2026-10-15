#include "main.h"
#include "lemlib/api.hpp"
#include "robot/robot-config.h"
#include "robot/odometry.h"
#include "robot/auto-pid.h"
#include <functional>
#include <new>

// drive PID settings + gains (from odometry.cpp)
extern lemlib::ControllerSettings lateral_controller;
extern lemlib::ControllerSettings angular_controller;
// latest tuned gains (start from whatever the controllers in odometry.cpp are set to)
double l_kP = 0, l_kI = 0, l_kD = 0, a_kP = 0, a_kI = 0, a_kD = 0;
extern pros::Rotation horizontal_sensor;

/* SETTING PID GAINS */
// lemlib::PID stores its gains as const, so the PID gets rebuilt in place with the new gains
// (same windup range + sign flip reset lemlib uses). only call these when no motion is running
void set_lateral_gains(double kP, double kI, double kD){
    lateral_controller.kP = kP;
    lateral_controller.kI = kI;
    lateral_controller.kD = kD;
    chassis.lateralPID.~PID();
    new (&chassis.lateralPID) lemlib::PID(kP, kI, kD, lateral_controller.windupRange, true);
}

void set_angular_gains(double kP, double kI, double kD){
    angular_controller.kP = kP;
    angular_controller.kI = kI;
    angular_controller.kD = kD;
    chassis.angularPID.~PID();
    new (&chassis.angularPID) lemlib::PID(kP, kI, kD, angular_controller.windupRange, true);
}

/* AUTO PID TUNER */
// what one run (turn or drive) looked like
struct PIDTrial{
    double exit_time = 0;       // ms until lemlib ended the motion (how long it'd block an auton)
    double overshoot = 0;       // furthest the robot went past the target
    int crossings = 0;          // times the robot crossed over the target
    double final_error = 0;     // error once stopped (+ = short of the target, - = past it)
    bool timed_out = false;     // motion hit its timeout instead of settling (usually oscillating)
    double rise_time = -1;      // ms until the robot got within 10% of the target (-1 = never did)
    bool stalled = false;       // robot barely moved (gains too weak to get going)
    double score = 0;           // lower is better
};

struct PIDGains{
    double kP, kI, kD;
};

// limits for tuning one controller
struct TuneSettings{
    const char* name;
    int runs;               // runs per set of gains (averaged so one bad run doesn't throw it off)
    double tolerance;       // how close counts as "at the target" (matches the small error range)
    double band;            // noise band so jitter around the target doesn't count as a crossing
    double min_kP, max_kP;  // min & max kP
    double max_kD;          // max kD
    int timeout;            // ms per motion
    int max_tests;          // most gain sets to try
    double ramp_step;       // kP multiplier per step while ramping up
    double ramp_kD_ratio;   // kD = kP * this while ramping (keeps the ramp damped)
    double fast_time;       // ms to get within 10% of the target that counts as "fast enough"
    double kP_flex;         // 0 = ramp/search kP freely. otherwise kP stays within start/flex..start*flex
};

bool tune_aborted = false;

// stops the tuner and shows why on the terminal, brain, and controller
void tune_error(const char* msg){
    tune_aborted = true;
    chassis.cancelAllMotions();
    printf("[auto_PID] ERROR: %s\n", msg);
    fflush(stdout);
    pros::lcd::print(7, "ERROR: %s", msg);
    controller.clear();
    pros::delay(60);
    controller.print(0, 0, "AUTO PID ERROR");
    pros::delay(60);
    controller.print(1, 0, "%.19s", msg);
    pros::delay(60);
}

/**
 * @brief checks the sensors the tuner needs are plugged in and the IMU calibrated
 * (chassis.calibrate() runs in initialize()) (so the robot can't run away
 * because odometry isn't updating)
 */
bool start_odom(bool need_tracking_wheel){
    if(!imu.is_installed()){
        tune_error("no IMU on its port");
        return false;
    }
    if(need_tracking_wheel && !horizontal_sensor.is_installed()){
        tune_error("no tracking wheel sensor");
        return false;
    }

    double heading = imu.get_heading();
    if(std::isnan(heading) || std::isinf(heading)){
        tune_error("IMU didn't calibrate");
        return false;
    }
    return true;
}

/**
 * @brief scores a run, lower is better
 *
 * Based on how long the motion blocks for (what actually matters in an auton), plus
 * penalties for overshooting and ending off target. The penalties are squared so small
 * misses barely count but big ones count a lot (1 tolerance off = +250ms, 2 off = +1000ms).
 */
double score_trial(PIDTrial trial, TuneSettings settings){
    double error = trial.final_error / settings.tolerance;
    double overshoot = trial.overshoot / settings.tolerance;
    // ringing back and forth is what makes an auton unreliable, so each extra crossing costs a lot
    double ringing = 300 * std::max(0, trial.crossings - 1);
    return trial.exit_time + 250 * error * error + 200 * overshoot * overshoot + ringing;
}

/**
 * @brief samples the error of the running motion every 10ms until the robot stops
 *
 * Keeps recording 250ms after lemlib exits the motion so coasting past the target
 * still counts as overshoot. Pressing B on the controller cancels the tuner, and it
 * also stops itself if odometry says the robot isn't moving or went way off course.
 */
PIDTrial record_trial(std::function<double()> get_error, double distance, TuneSettings settings){
    PIDTrial trial;
    int side = 1;
    int elapsed = 0;
    int stopped_at = -1;
    double error = get_error();

    while(elapsed < settings.timeout + 500){
        error = get_error();

        // only count a crossing once the error clears the noise band on the other side
        if(side == 1 && error < -settings.band){
            side = -1;
            trial.crossings++;
        }else if(side == -1 && error > settings.band){
            side = 1;
            trial.crossings++;
        }
        trial.overshoot = std::max(trial.overshoot, -error);

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            tune_aborted = true;
            chassis.cancelAllMotions();
            printf("[auto_PID] stopped with B\n");
            break;
        }
        if(trial.rise_time < 0 && error <= distance * 0.1) trial.rise_time = elapsed;

        // gains too weak to move the robot: not an error, the kP ramp just needs to go higher
        if(elapsed == 400 && error > distance * 0.97){
            trial.stalled = true;
            chassis.cancelAllMotions();
            break;
        }
        // way past the target: gains are clearly too high, cut the motion instead of letting it swing
        if(-error > settings.tolerance * 10){
            chassis.cancelAllMotions();
        }
        if(std::abs(error) > distance * 2){
            tune_error("odom is way off");
            break;
        }

        if(stopped_at < 0 && elapsed > 50 && !chassis.isInMotion()){
            stopped_at = elapsed;
        }
        if(stopped_at >= 0 && elapsed - stopped_at >= 250) break;

        pros::delay(10);
        elapsed += 10;
    }

    trial.exit_time = (stopped_at >= 0) ? stopped_at : elapsed;
    trial.timed_out = stopped_at < 0 || stopped_at >= settings.timeout - 20;
    trial.final_error = error;
    trial.score = score_trial(trial, settings);
    return trial;
}

// does every run for one set of gains and combines them (worst overshoot, average everything else)
PIDTrial run_trials(std::function<PIDTrial(int)> run_trial, int runs){
    PIDTrial total;
    int done = 0;

    for(int i = 0; i < runs && !tune_aborted; i++){
        PIDTrial trial = run_trial(i);
        total.exit_time += trial.exit_time;
        total.overshoot = std::max(total.overshoot, trial.overshoot);
        total.crossings = std::max(total.crossings, trial.crossings);
        total.final_error += trial.final_error;
        total.timed_out = total.timed_out || trial.timed_out;
        total.score += trial.score;
        done++;
    }

    if(done > 0){
        total.exit_time /= done;
        total.final_error /= done;
        total.score /= done;
    }
    return total;
}

void log_trial(TuneSettings settings, PIDGains gains, PIDTrial trial){
    const char* verdict = trial.timed_out ? "TIMED OUT" : (trial.crossings >= 2 ? "oscillating" : "ok");

    printf("[%s] kP %.3f kI %.4f kD %.3f | exit %.0fms overshoot %.2f error %.2f crossings %d %s | score %.0f\n",
           settings.name, gains.kP, gains.kI, gains.kD,
           trial.exit_time, trial.overshoot, trial.final_error, trial.crossings, verdict, trial.score);

    pros::lcd::print(2, "%s: kP %.2f  kI %.3f  kD %.2f", settings.name, gains.kP, gains.kI, gains.kD);
    pros::lcd::print(3, "exit %.0fms  overshoot %.2f", trial.exit_time, trial.overshoot);
    pros::lcd::print(4, "error %.2f  %s  score %.0f", trial.final_error, verdict, trial.score);
}

// prints the values to the terminal (save them with: pros terminal --output pid_values.txt)
// and to the controller screen as kP kI kD
void print_PID_values(const char* status){
    printf("\n// auto_PID: %s\n", status);
    printf("lateral: kP = %.3f, kI = %.4f, kD = %.3f\n", l_kP, l_kI, l_kD);
    printf("angular: kP = %.3f, kI = %.4f, kD = %.3f\n", a_kP, a_kI, a_kD);
    printf("double l_kP = %.3f, l_kI = %.4f, l_kD = %.3f, a_kP = %.3f, a_kI = %.4f, a_kD = %.3f;\n\n",
           l_kP, l_kI, l_kD, a_kP, a_kI, a_kD);
    fflush(stdout);

    // controller needs 50ms between prints
    controller.clear();
    pros::delay(60);
    controller.print(0, 0, "A %.2f %.3f %.1f", a_kP, a_kI, a_kD);
    pros::delay(60);
    controller.print(1, 0, "L %.2f %.3f %.1f", l_kP, l_kI, l_kD);
    pros::delay(60);
    controller.print(2, 0, "%s", status);
    pros::delay(60);
}

/**
 * @brief tunes one PID controller by running the same motion over and over
 *
 * kP and kD get tuned together with a pattern search: from the current best gains, try
 * raising/lowering kP, kD, or both, and move to whichever scores better. When nothing
 * nearby is better, the step size shrinks to fine tune, until the steps are tiny. This
 * handles kP and kD being way off (lemlib's kD especially, since its derivative is per
 * 10ms loop) and doesn't get fooled by one lucky run. kI is only added at the end if
 * the robot still keeps stopping short.
 *
 * @return whether the gains changed from where they started
 */
bool tune_controller(PIDGains& gains, TuneSettings settings,
                     std::function<void(PIDGains)> apply_gains, std::function<PIDTrial(int)> run_trial){
    int tests = 0;

    // runs a set of gains and returns how it did
    auto test_gains = [&](PIDGains test){
        apply_gains(test);
        PIDTrial trial = run_trials(run_trial, settings.runs);
        tests++;
        if(!tune_aborted) log_trial(settings, test, trial);
        return trial;
    };

    // kP held near where it started: no ramp, only kD (and a small kP trim) get tuned
    bool kP_locked = settings.kP_flex > 0;
    if(kP_locked){
        settings.min_kP = gains.kP / settings.kP_flex;
        settings.max_kP = gains.kP * settings.kP_flex;
    }

    PIDGains start = {std::clamp(gains.kP, settings.min_kP, settings.max_kP), 0, gains.kD};
    if(start.kD <= 0) start.kD = start.kP;
    PIDGains best = start;
    PIDTrial best_trial = test_gains(best);
    double best_score = best_trial.score;

    // ramp kP up (kD scaled with it) until the motion is fast, or it starts overshooting /
    // oscillating, which means the last kP was as high as it can go. runs a single motion per
    // step (run 0) since only the speed matters here. the old way jumped around and could quit
    // early; this keeps going until the robot is actually moving at a good speed
    double ramp_kP = 0;
    bool moved = false;
    for(double kP = settings.min_kP; !kP_locked && kP <= settings.max_kP && !tune_aborted; kP *= settings.ramp_step){
        PIDGains test = {kP, 0, std::min(kP * settings.ramp_kD_ratio, settings.max_kD)};
        apply_gains(test);
        PIDTrial trial = run_trial(0);
        if(tune_aborted) break;
        log_trial(settings, test, trial);

        if(trial.stalled) continue; // too weak to move, keep raising

        moved = true;
        if(trial.crossings >= 2 || trial.overshoot > settings.tolerance * 3) break; // too aggressive
        ramp_kP = kP;
        if(trial.rise_time >= 0 && trial.rise_time <= settings.fast_time) break;    // fast enough
    }

    if(kP_locked) ramp_kP = gains.kP;
    if(!tune_aborted && !moved && !kP_locked){
        tune_error("robot isn't moving");
    }
    if(tune_aborted){
        apply_gains(gains);
        return false;
    }
    if(ramp_kP <= 0) ramp_kP = settings.min_kP; // even the weakest gains overshot

    // now find the kD that goes with that kP: doubling until it stops overshooting (more
    // kD after that just slows it down)
    for(double ratio = 1; ratio <= 48 && !tune_aborted; ratio *= 2){
        PIDGains test = {ramp_kP, 0, std::min(ramp_kP * ratio, settings.max_kD)};
        PIDTrial trial = test_gains(test);
        if(tune_aborted) break;

        if(trial.score < best_score){
            best = test;
            best_trial = trial;
            best_score = trial.score;
        }
        if((trial.overshoot <= settings.tolerance && trial.crossings < 2) || test.kD >= settings.max_kD) break;
    }

    // directions to step in: {kP, kD}, 1 = raise, -1 = lower, 0 = leave alone
    const int directions[6][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}, {1, 1}, {-1, -1}};
    int first = 0;
    double step = 1.6;

    while(step > 1.07 && tests < settings.max_tests && !tune_aborted){
        bool improved = false;

        for(int i = 0; i < 6 && tests < settings.max_tests && !tune_aborted; i++){
            int d = (first + i) % 6;
            PIDGains test = best;
            test.kP = std::clamp(best.kP * std::pow(step, directions[d][0]), settings.min_kP, settings.max_kP);
            test.kD = std::clamp(best.kD * std::pow(step, directions[d][1]), 0.0, settings.max_kD);
            if(test.kP == best.kP && test.kD == best.kD) continue; // stuck on a limit

            PIDTrial trial = test_gains(test);
            if(tune_aborted) break;

            // needs to be clearly better (2%) so noise doesn't count as an improvement
            if(trial.score < best_score * 0.98){
                best = test;
                best_trial = trial;
                best_score = trial.score;
                first = d; // try the same direction first next time
                improved = true;
                break;
            }
        }

        if(!improved && !tune_aborted){
            // nothing nearby was better: rerun the best gains so one lucky run can't hold
            // the search back, then take smaller steps
            if(tests < settings.max_tests){
                PIDTrial again = test_gains(best);
                best_score = (best_score + again.score) / 2;
            }
            step = std::sqrt(step);
        }
    }

    // kI only if the robot still consistently stops short of the target
    if(!tune_aborted && best_trial.final_error > settings.tolerance * 0.5){
        PIDGains test = best;
        test.kI = best.kP * 0.02;

        for(int tries = 0; tries < 4 && !tune_aborted; tries++){
            PIDTrial trial = test_gains(test);
            if(tune_aborted) break;

            if(trial.score < best_score * 0.98){
                best = test;
                best_trial = trial;
                best_score = trial.score;
            }else if(best.kI > 0){
                break; // more kI made it worse
            }
            test.kI *= 2;
        }
    }

    // safety margin: gains right on the edge of ringing will ring on some angle that wasn't
    // tested, so back kP off a bit and keep that if it's about as fast or rings less
    if(!tune_aborted){
        PIDGains safe = best;
        safe.kP = std::max(best.kP * 0.85, settings.min_kP);
        if(safe.kP < best.kP){
            PIDTrial trial = test_gains(safe);
            if(!tune_aborted && (trial.crossings < best_trial.crossings || trial.score < best_score * 1.15)){
                best = safe;
                best_trial = trial;
                best_score = trial.score;
            }
        }
    }

    bool changed = best.kP != gains.kP || best.kI != gains.kI || best.kD != gains.kD;
    gains = best;
    apply_gains(gains);
    return changed;
}

// turns done for each set of angular gains. mostly 90, but tuning on only 90 makes gains
// that are great at 90 and get stuck a few degrees off on smaller / bigger turns
const double ANGULAR_TURNS[] = {90, 180, 45, 20, 135};
const int ANGULAR_TURN_COUNT = sizeof(ANGULAR_TURNS) / sizeof(ANGULAR_TURNS[0]);

/**
 * @brief tunes the angular PID by turning 90 (plus a 180 and 45) over and over
 *
 * The tuned values are applied to the chassis, shown on the controller, and every
 * run + the final values are printed to the terminal. To save them to a file on the
 * computer, run "pros terminal --output pid_values.txt" while the brain or controller
 * is plugged in. Hold B on the controller to stop (the best values so far still get
 * printed, and auto_lateral_PID won't run after).
 */
void auto_angular_PID(){
    if(tune_aborted || !start_odom(false)) return;

    pros::lcd::print(0, "auto PID: tuning angular  (B = stop)");
    TuneSettings settings = {"angular", 5, 1, 0.5, 1, 30, 450, 2000, 40, 1.4, 4, 400, 0};
    a_kP = angular_controller.kP;
    a_kI = angular_controller.kI;
    a_kD = angular_controller.kD;
    PIDGains gains = {a_kP, a_kI, a_kD};

    tune_controller(gains, settings,
        [](PIDGains gains){
            set_angular_gains(gains.kP, gains.kI, gains.kD);
        },
        [settings](int run){
            double angle = ANGULAR_TURNS[run % ANGULAR_TURN_COUNT];
            pros::delay(250); // let the robot come to rest
            chassis.setPose(0, 0, 0);
            chassis.turnToHeading(angle, settings.timeout);
            return record_trial([angle](){ return angle - chassis.getPose().theta; }, angle, settings);
        });

    a_kP = gains.kP;
    a_kI = gains.kI;
    a_kD = gains.kD;
    pros::lcd::print(0, "auto PID: angular %s", tune_aborted ? "stopped" : "done");
    print_PID_values(tune_aborted ? "angular stopped" : "angular done");
}

/**
 * @brief tunes the lateral PID by driving 48in forward, turning 180, and driving the
 * 48in back, over and over (needs ~5ft of open space in front of the robot)
 *
 * Run auto_angular_PID first, since the 180 turns and heading correction use the angular PID.
 * Results get printed + shown the same way as auto_angular_PID.
 */
void auto_lateral_PID(){
    if(tune_aborted || !start_odom(true)) return;

    pros::lcd::print(0, "auto PID: tuning lateral  (B = stop)");
    TuneSettings settings = {"lateral", 2, 1, 0.3, 2, 60, 600, 3000, 40, 1.4, 4, 800, 1.15};
    l_kP = lateral_controller.kP;
    l_kI = lateral_controller.kI;
    l_kD = lateral_controller.kD;
    PIDGains gains = {l_kP, l_kI, l_kD};

    tune_controller(gains, settings,
        [](PIDGains gains){
            set_lateral_gains(gains.kP, gains.kI, gains.kD);
        },
        [settings](int run){
            pros::delay(250);
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, 48, settings.timeout);
            PIDTrial trial = record_trial([](){ return 48 - chassis.getPose().y; }, 48, settings);

            // turn around so the next run drives back over the same 48in: two separate 90s
            // (each waits to finish) instead of one 180
            if(!tune_aborted) chassis.turnToHeading(90, 1000, {}, false);
            if(!tune_aborted) chassis.turnToHeading(180, 1000, {}, false);
            return trial;
        });

    l_kP = gains.kP;
    l_kI = gains.kI;
    l_kD = gains.kD;
    pros::lcd::print(0, "auto PID: lateral %s", tune_aborted ? "stopped" : "done");
    print_PID_values(tune_aborted ? "lateral stopped" : "lateral done");
}
