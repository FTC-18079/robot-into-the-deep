package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
import org.firstinspires.ftc.teamcode.vision.ATVision;

@Config
public class RobotCore extends Robot {
    static Telemetry telemetry;
    GamepadEx driveController;
    GamepadEx manipController;

    Pose initialPose;
    ATVision atVision;

    // Subsystems
    Chassis chassis;

    // Commands
    TeleOpDriveCommand driveCommand;

    // Drive values
    public static double DRIVE_SENSITIVITY = 1.1;
    public static double ROTATIONAL_SENSITIVITY = 2.0;
    public static double JOYSTICK_DEADZONE = 0.09;
    public static double TRIGGER_DEADZONE = 0.05;
    
    // Touchpad button
    Trigger touchpadTrigger;
  
    // Loop times
    private double loopTime = 0.0;
    private final ElapsedTime timer = new ElapsedTime();
    private double endTime = 0;

    // OpMode type enumerator
    public enum OpModeType {
        TELEOP, AUTO, EMPTY
    }

    public RobotCore(OpModeType type, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        RobotCore.telemetry = telemetry;
        touchpadTrigger = new Trigger(() -> getTouchpad(driveController));

        telemetry.addData("Status", "Initializing AprilTag detection");
        telemetry.update();
        atVision = new ATVision(true);

        telemetry.addData("Status", "Starting camera livestream");
        telemetry.update();
        FtcDashboard.getInstance().startCameraStream(atVision.livestream, 15);

        this.driveController = new GamepadEx(gamepad1);
        this.manipController = new GamepadEx(gamepad2);

        this.initialPose = RobotGlobal.robotPose;

        telemetry.addData("Status", "Initializing subsystems");
        telemetry.update();
        initSubsystems();

        // Set up opmode
        setupOpMode(type);

        telemetry.addData("Status", "Robot initialized, ready to enable");
        telemetry.update();
    }

    public void initSubsystems() {
        chassis = Chassis.getInstance();

        register(chassis);
    }

    public void setupOpMode(OpModeType type) {
        if (type != OpModeType.TELEOP) {
            telemetry.addData("Status", "Generating auto path");
            telemetry.update();
        }
        switch (type) {
            case TELEOP:
                chassis.startTeleopDrive();
                setDriveControls();
                break;
            case EMPTY:
                schedule(new InstantCommand());
                break;
        }
    }

    public void setDriveControls() {
        // Drive command
        driveCommand = new TeleOpDriveCommand(
                chassis,
                () -> responseCurve(driveController.getLeftY(), DRIVE_SENSITIVITY),
                () -> responseCurve(driveController.getLeftX(), DRIVE_SENSITIVITY),
                () -> responseCurve(driveController.getRightX(), ROTATIONAL_SENSITIVITY)
        );

        driveController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(chassis::resetHeading);
        driveController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(chassis::toggleRobotCentric);

        chassis.setDefaultCommand(driveCommand);
    }

    public double responseCurve(double value, double power) {
        return value * Math.pow(Math.abs(value), power - 1);
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();

        double loop = System.nanoTime();
        telemetry.addData("AprilTag FPS", atVision.getFPS());
        telemetry.addData("hz", 1000000000 / (loop - loopTime));
        telemetry.addData("Runtime", endTime == 0 ? timer.seconds() : endTime);
        loopTime = loop;

        telemetry.update();
    }

    public Pose getPoseEstimate() {
        return chassis.getPoseEstimate();
    }

    public static Telemetry getTelemetry() {
        return telemetry;
    }

    public void followPath(Path path) {
        chassis.followPath(path);
    }

    public void breakFollowing() {
        chassis.breakFollowing();
    }

    public boolean isBusy() {
        return chassis.isBusy();
    }

    public double getFPS() {
        return atVision.getFPS();
    }

    public void rumbleGamepad(GamepadEx gamepad, double rumble1, double rumble2, int ms) {
        gamepad.gamepad.rumble(rumble1, rumble2, ms);
    }

    public boolean getTouchpad(GamepadEx gamepad) {
        return gamepad.gamepad.touchpad;
    }

    public boolean getPS(GamepadEx gamepad) {
        return gamepad.gamepad.ps;
    }
}
