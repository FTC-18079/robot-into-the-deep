package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.RobotStatus.Alliance.*;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hydra;
import org.firstinspires.ftc.teamcode.RobotStatus;
import org.firstinspires.ftc.teamcode.chassis.Chassis;

@Config
@TeleOp(name = "Teleop Solo", group = "A")
public class TeleOpModeSolo extends LinearOpMode {
    public static double VELOCITY_MULTIPLIER = 0.6;

    private final Hydra robot = Hydra.getInstance();
    private boolean lastSquare = false;

    @Override
    public void runOpMode() {
        // INIT
        // Pass in the same gamepad twice to register as single driver
        robot.teleopInit(telemetry, hardwareMap, gamepad1, gamepad1);

        // INIT LOOP
        while (opModeInInit()) {
            // Allow for manual toggling of alliance
            if (gamepad1.square != lastSquare && gamepad1.square) {
                switch(RobotStatus.alliance) {
                    case NONE:
                    case RED:
                        RobotStatus.alliance = BLUE;
                        break;
                    case BLUE:
                        RobotStatus.alliance = RED;
                        break;
                }
            }
            lastSquare = gamepad1.square;

            telemetry.addData("Alliance", RobotStatus.alliance);
            telemetry.addData("Status", RobotStatus.robotState);
            telemetry.update();
        }

        // START
        gamepad1.setLedColor(1, 1, 0, -1);
        gamepad2.setLedColor(1, 1, 0, -1);

        RobotStatus.autoRan = RobotStatus.AutoRan.NONE;

        Chassis.getInstance().setVelocityMultiplier(VELOCITY_MULTIPLIER);

        // LOOP
        while (opModeIsActive()) {
            robot.periodic();
        }

        // END
        robot.disabledInit();
    }
}
