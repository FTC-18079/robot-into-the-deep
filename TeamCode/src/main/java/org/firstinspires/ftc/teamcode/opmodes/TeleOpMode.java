package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.RobotStatus.Alliance.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hydra;
import org.firstinspires.ftc.teamcode.RobotStatus;

@TeleOp(name = "Teleop", group = "A")
public class TeleOpMode extends LinearOpMode {
    private final Hydra hydra = Hydra.getInstance();
    private boolean lastSquare = false;

    @Override
    public void runOpMode() {
        // INIT
        hydra.teleopInit(telemetry, hardwareMap, gamepad1, gamepad2);

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

        // LOOP
        while (opModeIsActive()) {
            hydra.run();
        }

        // END
        RobotStatus.robotState = RobotStatus.RobotState.DISABLED;
    }
}
