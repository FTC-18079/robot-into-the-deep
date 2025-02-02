## TeamCode Module

This module contains all the actual robot code

## Packages
- [`arm`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/arm) - contains all functionality and commands for the pivoting arm and slides
- [`autonomous`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/autonomous) - contains our auto sequences
- [`chassis`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/chassis) - contains all functionality and commands for the chassis (glorified follower wrapper)
- [`TeamCode/src/main/java/org/firstinspires/ftc/teamcode/claw`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/claw) - contains all functionality for the claw
- [`opmodes`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes) - you'll never guess
- [`pedroPathing`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/pedroPathing) - contains constants and tuners for PedroPathing
- [`util`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util) - contains utility classes for the robot
    - [`util/commands`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/commands) - contains additional commands and command factories not found in FTCLib
    - [`util/hardware`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/hardware) - contains hardware wrappers for motor caching
    - [`util/opmode`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/util/opmode) - contains all utility opmodes, mostly subsystem tests
- [`vision`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/vision) - contains vision functionality

## Robot Classes

Aside from tuning/testing OpModes, we have four actual competition OpModes:
- [`Hydra`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/Hydra.java) - serves as the main robot class, contains all subsystems and runs the robot
- [`RobotMap`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotMap.java) - contains all HardwareMap devices for the robot
- [`RobotStatus`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/RobotStatus.java) - contains static variables to keep track of universal states of the robot

## OpModes

Aside from tuning/testing OpModes, we have four actual competition OpModes:
- [`TeleOp`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/TeleOpMode.java)
- [`Sample Auto`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/autos/Auto_Left_4_0.java)
- [`Specimen Auto`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/autos/Auto_Right_0_4.java)
- [`Hybrid Auto`](TeamCode/src/main/java/org/firstinspires/ftc/teamcode/opmodes/autos/Auto_Left_3_1.java) (Note that this one has not been worked on or ran since LM2)
