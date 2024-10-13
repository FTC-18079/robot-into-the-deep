package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.command.button.Trigger;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.collector.Collector;
import org.firstinspires.ftc.teamcode.collector.commands.CollectorCommands;
import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.elevator.commands.ElevatorCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
import org.firstinspires.ftc.teamcode.util.commands.Commands;
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
    Collector collector;
    Elevator elevator;

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

    // OpMode type enumerator
    public enum OpModeType {
        TELEOP, AUTO, EMPTY
    }

    public static RobotCore INSTANCE = null;

    public static RobotCore getInstance() {
        return INSTANCE;
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
    }

    public void initSubsystems() {
        chassis = Chassis.getInstance();
        collector = Collector.getInstance();
        elevator = Elevator.getInstance();
        register(chassis, collector, elevator);

        telemetry.addData("Status", "Robot initialized, ready to enable");
        telemetry.update();

        INSTANCE = this;
    }

    public void setupOpMode(OpModeType type) {
        switch (type) {
            case TELEOP:
                chassis.setPosition(RobotGlobal.robotPose);
                chassis.startTeleopDrive();
                setDriveControls();
                break;
            case EMPTY:
                schedule(Commands.none());
                break;
        }
        // Init servos
        schedule(Commands.sequence(
                Commands.runOnce(elevator::closeClaw, elevator),
                Commands.runOnce(collector::release, collector),
                Commands.runOnce(elevator::closeDoor, elevator),
                Commands.runOnce(elevator::restBucket, elevator)
        ));
    }

    public void setDriveControls() {
        // Drive command
        driveCommand = new TeleOpDriveCommand(
                () -> responseCurve(driveController.getLeftY(), DRIVE_SENSITIVITY),
                () -> responseCurve(driveController.getLeftX(), DRIVE_SENSITIVITY),
                () -> responseCurve(driveController.getRightX(), ROTATIONAL_SENSITIVITY)
        );

        // Reset heading
        driveController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(chassis::resetHeading);
        // Toggle drive mode
        driveController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(chassis::toggleRobotCentric);

        // Scoring buttons
        manipController.getGamepadButton(GamepadKeys.Button.A)
                .whenPressed(ElevatorCommands.SCORE_COMMAND)
                .whenReleased(ElevatorCommands.RELEASE_COMMAND);
        manipController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(elevator::closeClaw);

        // Elevator position buttons
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(elevator::toRest);
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT)
                .whenPressed(elevator::toLow);
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(elevator::toHigh);

        // Toggle game piece types
        manipController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(elevator::toggleScoreType);

        // Collector control
        manipController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(CollectorCommands.TO_STOW.get());
        manipController.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(CollectorCommands.TO_COLLECTING.get());
        manipController.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(CollectorCommands.COLLECT.get());
        manipController.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(CollectorCommands.TO_PASSTHROUGH.get());
//        new Trigger(() -> manipController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
//                .whenActive(Commands.either(
//                        Commands.runOnce(() -> collector.setCollectorState(Collector.CollectorState.COLLECTING), collector),
//                        Commands.runOnce(collector::out, collector)
//                                .andThen(Commands.waitMillis(250)
//                                .andThen(Commands.runOnce(collector::stop, collector))
//                                .andThen(Commands.runOnce(elevator::closeDoor, elevator))),
//                        () -> collector.getCollectorState() == Collector.CollectorState.SEEKING
//                ));

        // Deploy collector
//        manipController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(Commands.either(
//                        Commands.runOnce(() -> collector.setCollectorState(Collector.CollectorState.SEEKING), collector).andThen(Commands.runOnce(elevator::openDoor, elevator)),
//                        Commands.runOnce(() -> collector.setCollectorState(Collector.CollectorState.INACTIVE), collector),
//                        () -> collector.getCollectorState() == Collector.CollectorState.INACTIVE
//                ));

        // Toggle target color
        manipController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(Commands.runOnce(collector::toggleTargetColor, collector)
                        // Set gamepad colors cuz cool
                            .andThen(Commands.either(
                                    Commands.either(
                                            Commands.runOnce(() -> setControllerColors(1, 0, 0)),
                                            Commands.runOnce(() -> setControllerColors(0, 0, 1)),
                                            () -> RobotGlobal.alliance == RobotGlobal.Alliance.RED
                                    ),
                                    Commands.runOnce(() -> setControllerColors(1, 1, 0)),
                                    () -> collector.getTargetColor() != Collector.SampleColor.YELLOW
                            )));

        chassis.setDefaultCommand(driveCommand);
    }

    public double responseCurve(double value, double power) {
        return value * Math.pow(Math.abs(value), power - 1);
    }

    public static Telemetry getTelemetry() {
        return telemetry;
    }

    public boolean isBusy() {
        return chassis.isBusy();
    }

    public double getFPS() {
        return atVision.getFPS();
    }

    // Rumble drive controller
    public static void rumbleDrive(int ms) {
        INSTANCE.driveController.gamepad.rumble(1, 1, ms);
    }

    // Rumble manip controller
    public static void rumbleManip(int ms) {
        INSTANCE.manipController.gamepad.rumble(1, 1, ms);
    }

    // Change controller LEDs
    public void setControllerColors(double r, double g, double b) {
        driveController.gamepad.setLedColor(r, g, b, -1);
        manipController.gamepad.setLedColor(r, g, b, -1);
    }

    public boolean getTouchpad(GamepadEx gamepad) {
        return gamepad.gamepad.touchpad;
    }

    @Override
    public void run() {
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.clearBulkCache();
        }

        CommandScheduler.getInstance().run();

        double loop = System.nanoTime();
        telemetry.addLine();
        telemetry.addData("AprilTag FPS", atVision.getFPS());
        telemetry.addData("Loop Time", timer.milliseconds());
        telemetry.addData("hz", 1000000000 / (loop - loopTime));
        loopTime = loop;
        timer.reset();

        telemetry.update();
    }
}
