## TeamCode Module

This module contains all the actual robot code

## Packages
- [`arm`](arm) - contains all functionality and commands for the pivoting arm and slides
- [`autonomous`](autonomous) - contains our auto sequences
- [`chassis`](chassis) - contains all functionality and commands for the chassis (glorified follower wrapper)
- [`claw`](claw) - contains all functionality for the claw
- [`opmodes`](opmodes) - you'll never guess
- [`pedroPathing`](pedroPathing) - contains constants and tuners for PedroPathing
- [`util`](util) - contains utility classes for the robot
    - [`util/commands`](util/commands) - contains additional commands and command factories not found in FTCLib
    - [`util/hardware`](util/hardware) - contains hardware wrappers for motor caching
    - [`util/opmode`](util/opmode) - contains all utility opmodes, mostly subsystem tests
- [`vision`](vision) - contains vision functionality

## Robot Classes

Aside from tuning/testing OpModes, we have four actual competition OpModes:
- [`Hydra`](Hydra.java) - serves as the main robot class, contains all subsystems and runs the robot
- [`RobotMap`](RobotMap.java) - contains all HardwareMap devices for the robot
- [`RobotStatus`](RobotStatus.java) - contains static variables to keep track of universal states of the robot

## OpModes

Aside from tuning/testing OpModes, we have four actual competition OpModes:
- [`TeleOp`](opmodes/TeleOpMode.java)
- [`Sample Auto`](opmodes/autos/Auto_Left_4_0.java)
- [`Specimen Auto`](opmodes/autos/Auto_Right_0_4.java)
- [`Hybrid Auto`](opmodes/autos/Auto_Left_3_1.java) (Note that this one has not been worked on or ran since LM2)
