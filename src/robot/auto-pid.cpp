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
extern double l_kP, l_kI, l_kD, a_kP, a_kI, a_kD;

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
    double overshoot = 0;       // furthest the robot went past the target
    int crossings = 0;          // times the robot crossed over the target
    double settle_time = 0;     // ms until the robot stayed within tolerance
    double final_error = 0;     // error once stopped (+ = short of the target, - = past it)
    bool settled = false;       // whether the motion exited on its own within tolerance
};

struct PIDGains{
    double kP, kI, kD;
};

// limits for tuning one controller
struct TuneSettings{
    const char* name;
    double tolerance;       // how close counts as "at the target" (matches the small error range)
    double band;            // noise band so jitter around the target doesn't count as a crossing
    double max_overshoot;   // most overshoot allowed before kD gets raised
    double max_kP;
    double max_kD;
    int timeout;            // ms per motion
};

bool tune_aborted = false;

/**
 * @brief samples the error of the running motion every 10ms until the robot stops
 *
 * Keeps recording 250ms after lemlib exits the motion so coasting past the target
 * still counts as overshoot. Pressing B on the controller cancels the tuner.
 */
PIDTrial record_trial(std::function<double()> get_error, TuneSettings settings){
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
        if(std::abs(error) > settings.tolerance){
            trial.settle_time = elapsed + 10;
        }

        if(controller.get_digital(pros::E_CONTROLLER_DIGITAL_B)){
            tune_aborted = true;
            chassis.cancelAllMotions();
            break;
        }

        if(stopped_at < 0 && elapsed > 50 && !chassis.isInMotion()){
            stopped_at = elapsed;
        }
        if(stopped_at >= 0 && elapsed - stopped_at >= 250) break;

        pros::delay(10);
        elapsed += 10;
    }

    // running into the timeout means it never settled (usually oscillating)
    trial.final_error = error;
    trial.settled = stopped_at >= 0 && stopped_at < settings.timeout - 20 && std::abs(error) <= settings.tolerance;
    if(!trial.settled){
        trial.settle_time = elapsed;
    }
    return trial;
}

// runs the same motion twice and combines them so one bad run doesn't throw off the tuning
PIDTrial run_trials(std::function<PIDTrial()> run_trial){
    PIDTrial a = run_trial();
    if(tune_aborted) return a;
    PIDTrial b = run_trial();

    PIDTrial trial;
    trial.overshoot = std::max(a.overshoot, b.overshoot);
    trial.crossings = std::max(a.crossings, b.crossings);
    trial.settle_time = (a.settle_time + b.settle_time) / 2;
    trial.final_error = (a.final_error + b.final_error) / 2;
    trial.settled = a.settled && b.settled;
    return trial;
}

bool is_unstable(PIDTrial trial, TuneSettings settings){
    return trial.crossings >= 2 || trial.overshoot > settings.max_overshoot;
}

// lower is better: fast settling, little overshoot, ends on the target
double score_trial(PIDTrial trial, TuneSettings settings){
    return trial.settle_time
         + 150 * trial.overshoot / settings.tolerance
         + 400 * std::abs(trial.final_error) / settings.tolerance;
}

void log_trial(TuneSettings settings, PIDGains gains, PIDTrial trial){
    const char* verdict = is_unstable(trial, settings) ? "UNSTABLE" : (trial.settled ? "settled" : "slow");

    printf("[%s] kP %.3f kI %.4f kD %.3f | overshoot %.2f crossings %d settle %.0fms error %.2f %s\n",
           settings.name, gains.kP, gains.kI, gains.kD,
           trial.overshoot, trial.crossings, trial.settle_time, trial.final_error, verdict);

    pros::lcd::print(2, "%s: kP %.2f  kI %.3f  kD %.2f", settings.name, gains.kP, gains.kI, gains.kD);
    pros::lcd::print(3, "overshoot %.2f  crossings %d", trial.overshoot, trial.crossings);
    pros::lcd::print(4, "settle %.0fms  error %.2f  %s", trial.settle_time, trial.final_error, verdict);
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
 * Follows the usual tuning steps: raise kP until the robot oscillates / overshoots,
 * raise kD until that stops, and repeat until more kD can't fix it. The best stable
 * gains seen get kept. It does a coarse pass first, then a finer pass around the best
 * gains. kI is only added at the end if the robot still keeps stopping short.
 *
 * @return whether any stable gains were found (gains is left alone if not)
 */
bool tune_controller(PIDGains& gains, TuneSettings settings,
                     std::function<void(PIDGains)> apply_gains, std::function<PIDTrial()> run_trial){
    PIDGains test = {gains.kP, 0, gains.kD};
    PIDGains best = test;
    PIDTrial best_trial;
    double best_score = 1e9;
    bool found_stable = false;

    const double kP_step[] = {1.35, 1.1};
    const double kD_step[] = {1.3, 1.12};
    const int max_kD_bumps[] = {4, 3};
    const int max_tries[] = {14, 10};

    for(int pass = 0; pass < 2 && !tune_aborted; pass++){
        if(pass == 1){
            if(!found_stable) break;
            test = best;
        }
        int kD_bumps = 0;
        int no_improvement = 0;

        for(int tries = 0; tries < max_tries[pass] && !tune_aborted; tries++){
            apply_gains(test);
            PIDTrial trial = run_trials(run_trial);
            if(tune_aborted) break;
            log_trial(settings, test, trial);

            if(!is_unstable(trial, settings)){
                // stable: keep it if it's the best so far, then push kP higher
                double score = score_trial(trial, settings);
                if(score < best_score * 0.97){
                    no_improvement = 0;
                }else{
                    no_improvement++;
                }
                if(score < best_score){
                    best_score = score;
                    best = test;
                    best_trial = trial;
                }
                found_stable = true;
                kD_bumps = 0;

                // more kP isn't helping anymore (motors are probably maxed out)
                if(no_improvement >= 3 || test.kP >= settings.max_kP) break;
                test.kP = std::min(test.kP * kP_step[pass], settings.max_kP);
            }else if(kD_bumps < max_kD_bumps[pass] && test.kD < settings.max_kD){
                // oscillating or overshooting: more kD to dampen it
                test.kD = (test.kD <= 0) ? test.kP : std::min(test.kD * kD_step[pass], settings.max_kD);
                kD_bumps++;
            }else if(found_stable){
                // kD can't fix it anymore, so the best stable gains are as good as it gets
                break;
            }else{
                // nothing stable yet, back kP off and try again
                test.kP *= 0.6;
                kD_bumps = 0;
            }
        }
    }

    // kI only if the robot still consistently stops short of the target
    if(found_stable && !tune_aborted && best_trial.final_error > settings.tolerance){
        test = best;
        test.kI = best.kP * 0.02;

        for(int tries = 0; tries < 5 && !tune_aborted; tries++){
            apply_gains(test);
            PIDTrial trial = run_trials(run_trial);
            if(tune_aborted) break;
            log_trial(settings, test, trial);

            if(is_unstable(trial, settings)) break;
            if(std::abs(trial.final_error) <= settings.tolerance){
                best = test;
                break;
            }
            test.kI *= 2;
        }
    }

    if(found_stable){
        gains = best;
    }
    apply_gains(gains);
    return found_stable;
}

/**
 * @brief automatically tunes the angular and then the lateral PID
 *
 * Angular: turns 90 degrees over and over. Lateral: drives 48in forward, turns 180,
 * and drives the 48in back, over and over. Needs ~5ft of open space in front of
 * the robot. The tuned values are applied to the chassis, shown on the controller,
 * and every run + the final values are printed to the terminal. To save them to a
 * file on the computer, run "pros terminal --output pid_values.txt" while the brain
 * or controller is plugged in. Hold B on the controller to stop early (the best
 * values so far still get printed).
 */
void auto_PID(){
    tune_aborted = false;

    // starts odometry (robot must be still while the IMU calibrates)
    chassis.calibrate();

    // angular: turn 90 degrees over and over
    pros::lcd::print(0, "auto PID: tuning angular  (B = stop)");
    TuneSettings ang_settings = {"angular", 1, 0.5, 1.5, 20, 200, 1500};
    PIDGains ang_gains = {a_kP, a_kI, a_kD};

    bool ang_tuned = tune_controller(ang_gains, ang_settings,
        [](PIDGains gains){
            set_angular_gains(gains.kP, gains.kI, gains.kD);
        },
        [ang_settings](){
            pros::delay(250); // let the robot come to rest
            chassis.setPose(0, 0, 0);
            chassis.turnToHeading(90, ang_settings.timeout);
            return record_trial([](){ return 90 - chassis.getPose().theta; }, ang_settings);
        });

    a_kP = ang_gains.kP;
    a_kI = ang_gains.kI;
    a_kD = ang_gains.kD;
    pros::lcd::print(1, "angular: %s", ang_tuned ? "tuned" : "FAILED, kept old values");
    print_PID_values(tune_aborted ? "stopped (angular)" : (ang_tuned ? "angular done" : "angular failed"));

    if(tune_aborted || !ang_tuned){
        pros::lcd::print(0, "auto PID: stopped");
        return;
    }

    // lateral: drive 48in, turn 180, drive the 48in back, over and over
    pros::lcd::print(0, "auto PID: tuning lateral  (B = stop)");
    TuneSettings lat_settings = {"lateral", 1, 0.3, 1, 60, 300, 2500};
    PIDGains lat_gains = {l_kP, l_kI, l_kD};

    bool lat_tuned = tune_controller(lat_gains, lat_settings,
        [](PIDGains gains){
            set_lateral_gains(gains.kP, gains.kI, gains.kD);
        },
        [lat_settings](){
            pros::delay(250);
            chassis.setPose(0, 0, 0);
            chassis.moveToPoint(0, 48, lat_settings.timeout);
            PIDTrial trial = record_trial([](){ return 48 - chassis.getPose().y; }, lat_settings);

            // turn around so the next run drives back over the same 48in
            if(!tune_aborted) chassis.turnToHeading(180, 1500, {}, false);
            return trial;
        });

    l_kP = lat_gains.kP;
    l_kI = lat_gains.kI;
    l_kD = lat_gains.kD;
    pros::lcd::print(1, "lateral: %s", lat_tuned ? "tuned" : "FAILED, kept old values");
    print_PID_values(tune_aborted ? "stopped (lateral)" : (lat_tuned ? "done" : "lateral failed"));

    pros::lcd::print(0, "auto PID: %s", tune_aborted ? "stopped" : "done");
    pros::lcd::print(5, "A: %.2f %.3f %.2f  L: %.2f %.3f %.2f", a_kP, a_kI, a_kD, l_kP, l_kI, l_kD);
}
