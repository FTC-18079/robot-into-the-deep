package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.arm.Arm;
import org.firstinspires.ftc.teamcode.arm.commands.ArmCommands;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.chassis.commands.TeleOpDriveCommand;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.climb.Climb;
import org.firstinspires.ftc.teamcode.climb.commands.AscentTwoCommand;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;
import org.firstinspires.ftc.teamcode.util.commands.Commands;
import org.firstinspires.ftc.teamcode.vision.LLVision;

import java.util.ArrayList;
import java.util.List;

/**
 * This is our Robot class. It runs all subsystems and handles all functionality.
 * It functions as a singleton, so all references to it come from getInstance()
 */
public class Hydra extends Robot {
    private Telemetry telemetry = FtcDashboard.getInstance().getTelemetry();

    private final List<SubsystemIF> subsystems = new ArrayList<>();

    private static final Hydra INSTANCE = new Hydra();

    public static Hydra getInstance() {
        return INSTANCE;
    }

    private GamepadEx driveController;
    private GamepadEx manipController;

    private static final double DRIVE_SENSITIVITY = 1.1;
    private static final double ROTATIONAL_SENSITIVITY = 2.0;
    private static final double TRIGGER_DEADZONE = 0.1;

    private final ElapsedTime timer = new ElapsedTime();

    private Hydra() {
        reset();
        robotInit();
        Log.i("Hydra", "===============ROBOT CREATED SUCCESSFULLY===============");
    }

    // INITIALIZE

    // TODO: uncomment this and test if it creates robot on app startup
//    @OnCreateEventLoop
//    public static void onCreate(Context context) { getInstance(); }

    // Resets the robot and command scheduler
    @Override
    public void reset() {
        RobotStatus.robotState = RobotStatus.RobotState.DISABLED;
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
        Log.i("Hydra", "===============COMMAND SCHEDULER CLEARED===============");
    }

    // Registers all subsystems to the command scheduler
    private void registerSubsystems() {
        for (SubsystemIF s : subsystems) {
            register(s);
        }
    }

    // Initializes all subsystems and adds them to the list. New subsystems should be added here.
    private void robotInit() {
        subsystems.clear();
        subsystems.add(Chassis.getInstance().initialize());
        subsystems.add(Arm.getInstance().initialize());
        subsystems.add(Claw.getInstance().initialize());
        subsystems.add(LLVision.getInstance().initialize());
        subsystems.add(Climb.getInstance().initialize());
        registerSubsystems();
    }

    // Runs the process of disabling the robot
    public void disabledInit() {
        RobotStatus.robotState = RobotStatus.RobotState.DISABLED;
        telemetry = FtcDashboard.getInstance().getTelemetry();
        Log.i("Hydra", "===============ROBOT DISABLED===============");
    }

    // Runs autonomous initialization sequence
    public void autonomousInit(Telemetry telemetry, HardwareMap hardwareMap) {
        reset();
        registerSubsystems();
        RobotStatus.robotState = RobotStatus.RobotState.AUTONOMOUS_INIT;

        // Update telemetry and hardwareMap objects
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotMap.getInstance().init(hardwareMap);
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Run autonomous init
        subsystems.forEach(SubsystemIF::onAutonomousInit);
        Log.i("Hydra", "============INITIALIZED AUTONOMOUS============");
    }

    // Runs teleop initialization sequence and binds controls
    public void teleopInit(Telemetry telemetry, HardwareMap hardwareMap, Gamepad drive, Gamepad manip) {
        reset();
        registerSubsystems();
        RobotStatus.robotState = RobotStatus.RobotState.TELEOP_INIT;

        // Update telemetry and hardwareMap objects
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        RobotMap.getInstance().init(hardwareMap);
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Update gamepad objects
        driveController = new GamepadEx(drive);
        manipController = new GamepadEx(manip);

        // Run teleop init
        subsystems.forEach(SubsystemIF::onTeleopInit);

        // Chassis driving
        Chassis.getInstance().setDefaultCommand(new TeleOpDriveCommand(
                () -> applyResponseCurve(driveController.getLeftY(), DRIVE_SENSITIVITY),
                () -> applyResponseCurve(driveController.getLeftX(), DRIVE_SENSITIVITY),
                () -> applyResponseCurve(driveController.getRightX(), ROTATIONAL_SENSITIVITY)
        ));
        new Trigger(() -> driveController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
                .whenActive(Chassis.getInstance()::enableSlowMode)
                .whenInactive(Chassis.getInstance()::disableSlowMode);

        // Reset chassis heading
        driveController.getGamepadButton(GamepadKeys.Button.Y)
                .whenPressed(Commands.runOnce(() -> rumble(300, driveController)))
                .whenPressed(Chassis.getInstance()::resetHeading);

        // Toggle field centric
        driveController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(Chassis.getInstance()::toggleFieldCentric);

        // Arm to scoring position
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(ArmCommands.TO_CHAMBER)
                .whenPressed(ArmCommands.TO_BASKET);

        // Arm to stow
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(ArmCommands.TO_STOW);

        // Arm action
        new Trigger(() -> manipController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
                .whenActive(ArmCommands.ARM_ACTION)
                .whenInactive(Commands.either(
                        Commands.defer(ArmCommands.BASKET_TO_STOW),
                        Commands.none(),
                        () -> Arm.getInstance().getState() == Arm.ArmState.SCORING_SAMPLE
                ));

//        manipController.getGamepadButton(GamepadKeys.Button.START)
//                .whenPressed(Arm.getInstance()::resetPivotEncoder);
//                .whenPressed(new PivotZeroCommand());

        // Switch game pieces
        manipController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(Commands.runOnce(() -> rumble(300, manipController)))
                .whenPressed(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SAMPLE));
        manipController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(Commands.runOnce(() -> rumble(300, manipController)))
                .whenPressed(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SPECIMEN));

        // Claw overrides
        manipController.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(() -> LLVision.getInstance().setClawOverride(1.0));
        manipController.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(() -> LLVision.getInstance().setClawOverride(0.43));
        manipController.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(LLVision.getInstance()::disableClawOverride);

        // Toggle target color
        manipController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(Commands.runOnce(() -> rumble(300, manipController)))
                .whenPressed(Commands.either(
                        Commands.either(
                                Commands.runOnce(LLVision.getInstance()::setBlue).andThen(Commands.runOnce(() -> setGamepadColors(0, 0, 1))),
                                Commands.runOnce(LLVision.getInstance()::setRed).andThen(Commands.runOnce(() -> setGamepadColors(1, 0, 0))),
                                () -> RobotStatus.alliance == RobotStatus.Alliance.BLUE
                        ),
                        Commands.runOnce(LLVision.getInstance()::setYellow).andThen(Commands.runOnce(() -> setGamepadColors(1, 1, 0))),
                        () -> LLVision.getInstance().getTargetColor() == LLVision.SampleColor.YELLOW
                ));

        // Release sample
        manipController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(Commands.sequence(
                        Commands.runOnce(() -> Claw.getInstance().setJointOne(ClawConstants.JOINT_ONE_EJECT_POSITION)),
                        Commands.waitMillis(300),
                        Commands.runOnce(Claw.getInstance()::openClaw),
                        Commands.waitMillis(ClawConstants.GRAB_DELAY),
                        Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.REST_STATE))
                ));

        // Climb
        new Trigger(() -> manipController.gamepad.touchpad)
                .whenActive(Commands.either(
                        // Start climb
                        new AscentTwoCommand(),
                        // Check if climb is ready to finish
                        Commands.none(),
                        () -> Arm.getInstance().getState() == Arm.ArmState.STOW && RobotStatus.climbState == RobotStatus.ClimbState.DISABLED
                ));
//        manipController.getGamepadButton(GamepadKeys.Button.START)
//                .whenPressed(Commands.either(
//                        new AscentThreeCommand(),
//                        Commands.none(),
//                        () -> RobotStatus.climbState == RobotStatus.ClimbState.CLIMBED
//                ));

        Log.i("Hydra", "============INITIALIZED TELEOP============");
    }

    // GAMEPAD INTERACTION

    private double applyResponseCurve(double value, double power) {
        return value * Math.pow(Math.abs(value), power - 1);
    }

    /**
     * Rumbles controllers for a duration
     * @param ms how many milliseconds to rumble for
     * @param gamepads the controllers to rumble
     */
    private void rumble(int ms, GamepadEx... gamepads) {
        for(GamepadEx g : gamepads) {
            g.gamepad.rumble(1, 1, ms);
        }
    }

    /**
     * Changes controller LED colors
     * @param r the red value of the color
     * @param g the blue value of the color
     * @param b the green value of the color
     */
    private void setGamepadColors(double r, double g, double b) {
        driveController.gamepad.setLedColor(r, g, b, -1);
        manipController.gamepad.setLedColor(r, g, b, -1);
    }

    // GETTERS

    public Telemetry getTelemetry() {
        return telemetry;
    }

    // PERIODIC

    public void periodic() {
        // Update status out of init into enabled
        if (!RobotStatus.isEnabled() && RobotStatus.isTeleop()) RobotStatus.robotState = RobotStatus.RobotState.TELEOP_ENABLED;
        if (!RobotStatus.isEnabled() && !RobotStatus.isTeleop()) RobotStatus.robotState = RobotStatus.RobotState.AUTONOMOUS_ENABLED;

        // Clear hardware cache
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.clearBulkCache();
        }

        // Run the command scheduler
        run();

        // Log loop times
        telemetry.addLine();
        telemetry.addData("Status", RobotStatus.robotState);
        telemetry.addData("Loop Time", timer.milliseconds());
        telemetry.update();
        timer.reset();
    }
}
