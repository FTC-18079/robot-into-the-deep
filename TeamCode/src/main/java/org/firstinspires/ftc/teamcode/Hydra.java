package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.Command;
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
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
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
    private Telemetry telemetry;

    private final List<SubsystemIF> subsystems = new ArrayList<>();

    private static final Hydra INSTANCE = new Hydra();

    public static Hydra getInstance() {
        return INSTANCE;
    }

    private GamepadEx driveController;
    private GamepadEx manipController;

    public static double DRIVE_SENSITIVITY = 1.1;
    public static double ROTATIONAL_SENSITIVITY = 2.0;
    public static double TRIGGER_DEADZONE = 0.1;

    private final ElapsedTime timer = new ElapsedTime();

    private Hydra() {
        reset();
        robotInit();
    }

    // INITIALIZE

    @Override
    public void reset() {
        CommandScheduler.getInstance().reset();
        CommandScheduler.getInstance().cancelAll();
        CommandScheduler.getInstance().clearButtons();
    }

    public void robotInit() {
        subsystems.add(Chassis.getInstance());
        subsystems.add(Arm.getInstance());
        subsystems.add(Claw.getInstance());
        subsystems.add(LLVision.getInstance());
    }

    public void autonomousInit(Telemetry telemetry, HardwareMap hardwareMap) {
        reset();

        this.telemetry = telemetry;
        RobotMap.getInstance().init(hardwareMap);
    }

    public void teleopInit(Telemetry telemetry, HardwareMap hardwareMap, Gamepad drive, Gamepad manip) {
        reset();

        this.telemetry = telemetry;
        RobotMap.getInstance().init(hardwareMap);
        driveController = new GamepadEx(drive);
        manipController = new GamepadEx(manip);

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
                .whenPressed(rumble(300, driveController))
                .whenPressed(Chassis.getInstance()::resetHeading);

        // Toggle field centric
        driveController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(Chassis.getInstance()::toggleRobotCentric);

        // Arm to scoring position
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_UP)
                .whenPressed(ArmCommands.TO_CHAMBER)
                .whenPressed(ArmCommands.TO_BASKET);

        // Arm to stow
        manipController.getGamepadButton(GamepadKeys.Button.DPAD_DOWN)
                .whenPressed(ArmCommands.TO_STOW);

        // Arm action
        new Trigger(() -> manipController.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > TRIGGER_DEADZONE)
                .whenActive(ArmCommands.ARM_ACTION);

        // Switch game pieces
        manipController.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
                .whenPressed(rumble(300, manipController))
                .whenPressed(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SAMPLE));
        manipController.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER)
                .whenPressed(rumble(300, manipController))
                .whenPressed(() -> Arm.getInstance().setScoreType(Arm.ScoreType.SPECIMEN));

        // Claw overrides
        manipController.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
                .whenPressed(() -> LLVision.getInstance().setClawOverride(1.0));
        manipController.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
                .whenPressed(() -> LLVision.getInstance().setClawOverride(0.43));
        manipController.getGamepadButton(GamepadKeys.Button.BACK)
                .whenPressed(LLVision.getInstance()::disableClawOverride);

        // Zeroing
        // TODO: add if needed

        // Toggle target color
        manipController.getGamepadButton(GamepadKeys.Button.B)
                .whenPressed(rumble(300, manipController))
                .whenPressed(Commands.either(
                        Commands.either(
                                Commands.runOnce(LLVision.getInstance()::setBlue).andThen(setGamepadColors(0, 0, 1)),
                                Commands.runOnce(LLVision.getInstance()::setRed).andThen(setGamepadColors(1, 0, 0)),
                                () -> RobotGlobal.alliance == RobotGlobal.Alliance.BLUE
                        ),
                        Commands.runOnce(LLVision.getInstance()::setYellow).andThen(setGamepadColors(1, 1, 0)),
                        () -> LLVision.getInstance().getTargetColor() == LLVision.SampleColor.YELLOW
                ));

        // Release sample
        manipController.getGamepadButton(GamepadKeys.Button.X)
                .whenPressed(Commands.sequence(
                        Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.SPECIMEN_COLLECT_STATE)),
                        Commands.runOnce(Claw.getInstance()::closeClaw),
                        Commands.waitMillis(400),
                        Commands.runOnce(Claw.getInstance()::openClaw),
                        Commands.waitMillis(200),
                        Commands.runOnce(() -> Claw.getInstance().setState(ClawConstants.REST_STATE))
                ));
    }

    // GAMEPAD INTERACTION

    public double applyResponseCurve(double value, double power) {
        return value * Math.pow(Math.abs(value), power - 1);
    }

    /**
     * Rumbles controllers for a duration
     * @param ms how many milliseconds to rumble for
     * @param gamepads the controllers to rumble
     * @return an empty command
     */
    public Command rumble(int ms, GamepadEx... gamepads) {
        for(GamepadEx g : gamepads) {
            g.gamepad.rumble(1, 1, ms);
        }
        return Commands.none();
    }

    /**
     * Changes controller LED colors
     * @param r the red value of the color
     * @param g the blue value of the color
     * @param b the green value of the color
     * @return an empty command
     */
    public Command setGamepadColors(double r, double g, double b) {
        driveController.gamepad.setLedColor(r, g, b, -1);
        manipController.gamepad.setLedColor(r, g, b, -1);
        return Commands.none();
    }

    // GETTERS

    public Telemetry getTelemetry() {
        return telemetry;
    }

    // PERIODIC

    @Override
    public void run() {
        // Clear hardware cache
        for (LynxModule hub : RobotMap.getInstance().getLynxModules()) {
            hub.clearBulkCache();
        }

        // Run the command scheduler
        CommandScheduler.getInstance().run();

        // Log loop times
        telemetry.addLine();
        telemetry.addData("Loop Time", timer.milliseconds());
        telemetry.update();
        timer.reset();
    }
}
