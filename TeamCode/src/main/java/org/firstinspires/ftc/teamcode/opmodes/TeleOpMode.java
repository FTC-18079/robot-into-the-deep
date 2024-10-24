package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.util.RobotGlobal.Alliance.BLUE;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.Alliance.RED;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.collector.Collector;
import org.firstinspires.ftc.teamcode.collector.commands.CollectorCommands;
import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.pedroPathing.util.Drawing;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

//@Photon
@TeleOp(name = "TeleOp", group = "AAA")
public class TeleOpMode extends OpMode {
    RobotCore robot;
    boolean lastSquare = false;

    @Override
    public void init() {
        RobotMap.getInstance().init(hardwareMap);
        Chassis.resetInstance();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        robot = new RobotCore(RobotCore.OpModeType.TELEOP, telemetry, gamepad1, gamepad2);
        Collector.getInstance().toTeleop();
        Chassis.getInstance().setPosition(RobotGlobal.robotPose);
    }

    @Override
    public void init_loop() {
        // Allow for manual toggling of alliance
        if (gamepad1.square != lastSquare && gamepad1.square) {
            switch(RobotGlobal.alliance) {
                case NONE:
                case RED:
                    RobotGlobal.alliance = BLUE;
                    break;
                case BLUE:
                    RobotGlobal.alliance = RED;
                    break;
            }
        }
        lastSquare = gamepad1.square;

        telemetry.addData("Alliance", RobotGlobal.alliance);
        telemetry.addData("AprilTag FPS", robot.getFPS());
        telemetry.addData("Status", "Robot initialized, ready to enable");

        telemetry.addData("COLLECTOR POS", Collector.getInstance().getCurrentPosition());
        telemetry.addData("COLLECTOR TARGET POS", Collector.getInstance().getTargetPose());
        telemetry.addData("COLLECTOR OUTPUT", Collector.getInstance().getOutput());
        telemetry.update();
    }

    @Override
    public void start() {
        robot.schedule(
                Commands.runOnce(Elevator.getInstance()::closeClaw),
                Commands.runOnce(Collector.getInstance()::release),
                Commands.runOnce(Elevator.getInstance()::closeDoor),
                Commands.runOnce(Elevator.getInstance()::restBucket),
                CollectorCommands.TO_STOW.get()
        );
    }

    @Override
    public void loop() {
        robot.run();
    }
}
