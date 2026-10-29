# Override-285C

![PROS 4.2.2](https://img.shields.io/badge/PROS-4.2.2-orange)
![LemLib 0.5.6](https://img.shields.io/badge/LemLib-0.5.6-blue)
![License: MIT](https://img.shields.io/badge/license-MIT-green)

This is the code for **Ziggathan**, the robot VEX team 285C is running in the 2026-27 V5RC season, *Override*. It's C++ on PROS, and LemLib handles odometry and the basic drive motions.

We built a few things on top of LemLib this season:

- A motion chaining layer that works out exit speeds from the robot's real top speed and braking
- A voltage-based PID for the lift with gravity and friction feedforward, plus fault detection
- A tuner that finds the drive PID gains by running the robot over and over and scoring each try
- A touchscreen auton selector on the V5 brain

You can follow the season on [YouTube](https://www.youtube.com/@DiscobotsChaos) and [Instagram](https://www.instagram.com/discobotschronos/).

## Contents

- [Season log](#season-log)
- [The robot](#the-robot)
- [Driver controls](#driver-controls)
- [Autonomous routines](#autonomous-routines)
- [How the code works](#how-the-code-works)
- [Project layout](#project-layout)
- [Building and uploading](#building-and-uploading)
- [Roadmap](#roadmap)
- [License](#license)

## Season log

| Date | Event | Result |
| --- | --- | --- |
| Oct 17, 2026 | [Katy Botober RECF Achieve Robotics Competition](https://recfevents.org/Events/Details/590) (Katy, TX) | 4th place, tournament finalist, **Design Award** |
| Oct 31, 2026 | [GHOST's Halloween Override @ UT Austin](https://events.vex.com/robot-competitions/vex-robotics-competition/VE-V5-26-65564.html) (Austin, TX) | Upcoming |

Katy was our first event of the season. We made it all the way to finals before losing there, and we won the Design Award.

The week after Katy we tried out a ratchet claw. We've since gone back to a pneumatic claw for UT Austin and recalibrated the lift.

## The robot

Ziggathan runs a four-motor drive, a two-motor lift, and a pneumatic claw. Everything below matches what's set in [robot-config.cpp](src/robot/robot-config.cpp), [drivetrain.cpp](src/robot/drivetrain.cpp), and [odometry.cpp](src/robot/odometry.cpp).

### Drivetrain

| Spec | Value |
| --- | --- |
| Motors | 4x 11W, blue cartridge (600 rpm) |
| Gearing | 36:60, for 360 rpm at the wheels |
| Wheels | 3.25" omnis (new style) |
| Track width | 10.882" |

### Ports

**Motors**

| Device | Port(s) | Notes |
| --- | --- | --- |
| Left drive | 16, 17 | Blue cartridge |
| Right drive | 19, 20 | Blue cartridge, reversed |
| Lift | 11, 18 | Green cartridge (200 rpm), port 18 reversed |

**Sensors**

| Device | Port | Notes |
| --- | --- | --- |
| Inertial sensor (IMU) | 9 | Heading for odometry |
| Rotation sensor | 4 | Zeroed on startup |
| AI Vision (back) | 3 | |
| AI Vision (side) | 8 | |

**Pneumatics (3-wire ports)**

| Device | Port |
| --- | --- |
| Claw | H |
| Claw rotate | B |
| Two-bar | C |
| Brakes | D, E |
| Toggles | F, G |

The claw is the only piston used in driver control and autons right now. The others are set up in code and ready to be wired into controls.

## Driver controls

Ziggathan uses tank drive on the main controller.

| Input | What it does |
| --- | --- |
| Left stick (up/down) | Left side of the drive |
| Right stick (up/down) | Right side of the drive |
| Up arrow | Drive straight forward at 85% power |
| Down arrow | Drive straight backward at 85% power |
| L2 (hold) | Raise the lift |
| L1 (hold) | Lower the lift (slows down under 5") |
| R2 (hold) | Open the claw. Let go to close it. |

Two things happen without the driver doing anything:

- **Auto lift.** When the claw closes while the lift is all the way down, the lift raises itself to 2".
- **Lift hold.** When the driver lets go of L1/L2, the lift holds where it stopped.

The sticks go through LemLib's exponential drive curve with a deadband of 10 (out of 127).

## Autonomous routines

### Picking a routine

The selector shows up on the brain screen when the robot is plugged into a field controller. The first page has four buttons: **Left**, **Right**, **Skip**, and **Skills**. Picking Left or Right opens a second page with the routines for that side. If nothing gets picked, the robot runs Left Repeater.

### Routines

| Routine | Starting pose (x, y, heading) | What it does |
| --- | --- | --- |
| Left Repeater | (64.5, -46.35, 270°) | Scores the preloaded pin, then grabs two more from the match loader and scores them in the same goal. The last one goes in at 18" of lift height. |
| Right Repeater | (61.5, 56, 270°) | The same route on the other side of the field. |
| Skills | (64.5, -46.35, 270°) | Scores the preload, then runs five match loader cycles, dropping each pin 8" higher than the last to build a stack. |
| Skip | (0, 0, 0°) | Drives forward 5" and stops. |

Coordinates are in inches using LemLib's field frame. All routines are in [autos.cpp](src/robot/autos.cpp).

## How the code works

### Motion chaining

When a robot stops all the way between moves, it wastes time. LemLib can carry speed from one motion into the next with `minSpeed` and `earlyExitRange`, but picking those numbers by hand for every move gets old fast. `CustomChassis` in [odometry.h](include/robot/odometry.h) works them out for each motion.

It adds `chainToPoint`, `chainToPose`, and `chainToHeading`. Each one waits for the last motion to finish, then calls the matching LemLib motion with the chaining values filled in:

```cpp
chassis.chainToPoint(48, -26, 1200);
chassis.chainToHeading(90, 800);
chassis.chainToPoint(64, -56, 1000, {.forwards = false, .maxSpeed = 90});
```

For a move of distance `d`, the code first finds how fast the robot can actually get before it has to start braking. It keeps a fraction of that speed (the carry) going into the next motion, then figures out how far ahead of the target it needs to exit to still slow down in time:

```text
reachable  = min( sqrt(2 * accel * d), topSpeed * maxSpeed / 127 )
carry      = reachable * carryFraction
minSpeed   = carry / topSpeed * 127
exitRange  = min( carry^2 / (2 * accel), d / 2 )
```

`topSpeed` comes from the drivetrain settings (rpm, wheel size, track width), so the numbers stay right if the drive changes.

| Constant | Lateral | Angular |
| --- | --- | --- |
| Max deceleration | 150 in/s² | 1200 °/s² |
| Carry fraction | 0.5 | 0.25 |

Short moves get special handling. If a point is less than 15" away and the robot is facing more than 10° off, it turns to face the point first so it doesn't arc. Under 7.5", it decides whether to drive forward or backward based on which way it's already facing.

### Lift PID

The lift doesn't use a potentiometer. It measures height from the motor encoders, which get zeroed when the program starts. The relationship between motor degrees and inches isn't linear on our lift, so we measured 14 points by hand (0" up to 32.4") and the code interpolates between them.

The PID loop runs every 10 ms in its own task and sends voltage straight to the motors:

| Term | Value | What it's for |
| --- | --- | --- |
| kP | 3000 mV per inch | Main push toward the target |
| kI | 5000 mV per inch-second | Fixes small leftover error, only counts within 1" of the target |
| kD | 100 mV per in/s | Damps against the lift's speed (filtered) |
| kG | 150 mV | Constant hold against gravity when the target is above the floor |
| kS | 400 mV | Extra push to get past friction near the target |

The integral resets on every new target and gets cut to 30% when the lift crosses the target, so it doesn't overshoot. The output is also capped near the top and bottom of travel. When the target is 0", the lift pushes down with at least 4 V so it seats fully.

The lift also watches itself for problems. If it's pushing with more than 6 V for half a second and has moved less than an inch, it cuts power and prints the problem to the brain screen. That catches jams and unplugged motors before they overheat. If the lift moved the *wrong* way, the message says to flip `lift_motor_dir`.

The code is in [odometry.cpp](src/robot/odometry.cpp), and the driver-side helpers are in [functions.cpp](src/robot/functions.cpp).

### Drive PID auto-tuner

Tuning drive PID by hand takes a lot of guess-and-check. [auto-pid.cpp](src/robot/auto-pid.cpp) does that part for us. It runs the same motion over and over with different gains, scores each run, and keeps the best set.

Each run is scored like this (lower is better):

```text
score = exitTime(ms) + 250 * (finalError / tolerance)^2
                     + 200 * (overshoot / tolerance)^2
                     + 300 * (extra target crossings)
```

The squared terms mean a tiny miss barely counts, but a big one counts a lot.

The search goes through these steps:

1. **Ramp up kP.** Start low and multiply kP by 1.4 each step, keeping kD at 4x kP. Stop when the robot starts oscillating, overshoots badly, or gets close to the target fast enough.
2. **Find a kD.** With that kP, keep doubling kD until the overshoot is within tolerance.
3. **Pattern search.** Try nudging kP up, kD up, both up, and the reverse of each. Move to any neighbor that scores at least 2% better. When nothing nearby is better, shrink the step and try again, until the steps are tiny.
4. **Add kI if needed.** kI only gets added if the robot keeps stopping short.
5. **Leave some margin.** Try the result with 15% less kP and keep it if it does almost as well, since a slightly softer tune holds up better from match to match.

The angular tuner cycles through turns of 90°, 180°, 45°, 20°, and 135° and averages 5 runs per set of gains. The lateral tuner drives 48", turns around, and averages 2 runs per set. It only lets kP move within 15% of where it started.

For safety, the tuner checks that the IMU is plugged in and calibrated before starting. It stops right away if odometry reads something impossible or if the driver presses **B**. When it finishes, it prints the gains to the terminal as a line you can paste straight into the code, and shows them on the controller screen.

The tuner doesn't run by default. To use it, call `auto_angular_PID()` or `auto_lateral_PID()` from `opcontrol()` during a practice session.

Current drive PID values in [odometry.cpp](src/robot/odometry.cpp):

| Controller | kP | kI | kD | Slew |
| --- | --- | --- | --- | --- |
| Lateral | 6.5 | 0 | 10 | 20 |
| Angular | 2.75 | 0 | 14.9 | none |

### Auton selector

The selector uses a small `Button` class in [autos.h](include/robot/autos.h). Each button draws itself on the brain and checks for touches. It waits for the finger to lift before counting the press, so one tap can't select two things.

## Project layout

```text
override/
├── include/
│   ├── main.h
│   └── robot/
│       ├── robot-config.h   # controllers, motors, pistons, sensors
│       ├── drivetrain.h
│       ├── odometry.h       # CustomChassis with motion chaining
│       ├── functions.h      # driver helpers and lift helpers
│       ├── autos.h          # routines and the Button class
│       └── auto-pid.h       # drive PID tuner
├── src/
│   ├── main.cpp             # initialize, autonomous, opcontrol
│   └── robot/
│       ├── robot-config.cpp
│       ├── drivetrain.cpp
│       ├── odometry.cpp     # chassis setup, drive PID, lift PID
│       ├── functions.cpp
│       ├── autos.cpp
│       └── auto-pid.cpp
├── firmware/                # PROS kernel and LemLib libraries
├── project.pros
└── Makefile
```

## Building and uploading

You need [VS Code](https://code.visualstudio.com/) with the [PROS extension](https://marketplace.visualstudio.com/items?itemName=sigbots.pros). The PROS kernel (4.2.2) and LemLib (0.5.6) are already in the repo, so there's nothing else to install.

1. Clone the repo and open the `override` folder in VS Code.
2. The PROS extension picks up `project.pros` on its own.
3. Plug the brain or controller into your computer over USB.
4. In the PROS sidebar, click **Build and Upload**.

Before a match:

- **Lift all the way down when the program starts.** The lift encoders zero on startup, so if the lift is up, every height after that is off.
- **Keep the robot still for the first couple of seconds.** The IMU calibrates in `initialize()`, and moving it during calibration throws off the heading.

## Roadmap

- **Intake.** One 11W motor on port 15. It's written in `robot-config.cpp` but commented out until the mechanism is on the robot.
- **Toggle motors.** Two motors on ports 1 and 2, also commented out for now.
- **Horizontal tracking wheel.** A rotation sensor on a 2" omni, set up in `odometry.cpp` but turned off. Right now odometry runs on the IMU and drive encoders. The lateral PID tuner needs this wheel to run.

## License

Released under the [MIT License](LICENSE).
