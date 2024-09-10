package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class RobotCore extends Robot {
    Telemetry telemetry;
    GamepadEx driveController;
    GamepadEx manipController;
    Trigger touchpadTrigger;

    public RobotCore() {
        touchpadTrigger = new Trigger(() -> getTouchpad(driveController));
    }

    @Override
    public void run() {
        CommandScheduler.getInstance().run();
    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public boolean getTouchpad(GamepadEx gamepad) {
        return gamepad.gamepad.touchpad;
    }

    public boolean getPS(GamepadEx gamepad) {
        return gamepad.gamepad.ps;
    }
}
