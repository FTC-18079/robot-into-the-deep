package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.RobotStatus.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.RobotStatus.Alliance.RED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hydra;
import org.firstinspires.ftc.teamcode.RobotStatus;

@TeleOp(name = "Teleop", group = "AAA")
public class TeleOpMode extends OpMode {
    private final Hydra hydra = Hydra.getInstance();
    private boolean lastSquare = false;

    @Override
    public void init() {
        hydra.teleopInit(telemetry, hardwareMap, gamepad1, gamepad2);
    }

    @Override
    public void init_loop() {
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

    @Override
    public void start() {
        gamepad1.setLedColor(1, 1, 0, -1);
        gamepad2.setLedColor(1, 1, 0, -1);
    }

    @Override
    public void loop() {
        hydra.run();
    }
}
