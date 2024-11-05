package org.firstinspires.ftc.teamcode.claw;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;

public class Claw extends SubsystemBase {
    Telemetry telemetry;

    Servo claw;
    Servo wrist;
    Servo joint;

    ClawState state;

    public static Claw INSTANCE = null;

    public static Claw getInstance() {
        return INSTANCE;
    }

    public Claw() {
        claw = RobotMap.getInstance().CLAW;
        wrist = RobotMap.getInstance().WRIST;
        joint = RobotMap.getInstance().JOINT;

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
        claw.setPosition(0);
    }

    public void closeClaw() {
        claw.setPosition(1);
    }

    public void setWrist(double pos) {
        wrist.setPosition(pos);
    }

    public void setJoint(double pos) {
        joint.setPosition(pos);
    }

    // PERIODIC

    @Override
    public void periodic() {
        claw.setPosition(state.clawPos);
        wrist.setPosition(state.wristPos);
        joint.setPosition(state.jointPos);
    }
}
