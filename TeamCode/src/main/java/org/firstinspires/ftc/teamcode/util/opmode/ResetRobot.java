package org.firstinspires.ftc.teamcode.util.opmode;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.collector.Collector;
import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
import org.firstinspires.ftc.teamcode.vision.LLVision;

@TeleOp(name = "Reset Robot", group = "B")
public class ResetRobot extends OpMode {
    @Override
    public void init() {
        telemetry.addLine("Press start to reset robot");
    }

    @Override
    public void start() {
        RobotGlobal.resetValues();
        CommandScheduler.getInstance().reset();

        // Reset all subsystems
        Chassis.resetInstance();
        Collector.resetInstance();
        Elevator.resetInstance();
        LLVision.resetInstance();

        requestOpModeStop();
    }

    @Override
    public void loop() {
    }
}
