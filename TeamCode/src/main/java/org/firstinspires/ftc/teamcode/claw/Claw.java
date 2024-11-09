package org.firstinspires.ftc.teamcode.claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;

public class Claw extends SubsystemBase {
    Telemetry telemetry;

    Servo claw;
    Servo wrist;
    Servo jointOne;
    Servo jointTwo;

    ClawState state;

    public static Claw INSTANCE = null;

    public static Claw getInstance() {
        return INSTANCE;
    }

    public Claw() {
        claw = RobotMap.getInstance().CLAW;
        wrist = RobotMap.getInstance().WRIST;
        jointOne = RobotMap.getInstance().JOINT_ONE;
        jointTwo = RobotMap.getInstance().JOINT_TWO;

        state = new ClawState();
        INSTANCE = this;
    }

    // GETTERS

    public ClawState getState() {
        return state;
    }

    // SETTERS

    public void setState(ClawState state) {
        this.state = state;
    }

    public void openClaw() {
        state.clawPos = 0;
    }

    public void closeClaw() {
        state.clawPos = 1;
    }

    // PERIODIC

    @Override
    public void periodic() {
        claw.setPosition(state.clawPos);
        wrist.setPosition(state.wristPos);
        jointOne.setPosition(state.jointOnePos);
        jointTwo.setPosition(state.jointTwoPos);
    }
}
