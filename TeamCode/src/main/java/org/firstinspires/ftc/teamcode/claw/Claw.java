package org.firstinspires.ftc.teamcode.claw;

import static org.firstinspires.ftc.teamcode.claw.ClawConstants.*;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;

public class Claw extends SubsystemBase {
    Telemetry telemetry;

    Servo claw;
    Servo wrist;
    Servo jointOne;
    Servo jointTwo;

    ClawState state = new ClawState();

    public static Claw INSTANCE = null;

    public static Claw getInstance() {
        return INSTANCE;
    }

    public Claw() {
        claw = RobotMap.getInstance().CLAW;
        wrist = RobotMap.getInstance().WRIST;
        jointOne = RobotMap.getInstance().JOINT_ONE;
        jointTwo = RobotMap.getInstance().JOINT_TWO;

        telemetry = RobotCore.getTelemetry();
        state.clawPos = REST_STATE.clawPos;
        state.wristPos = REST_STATE.wristPos;
        state.jointOnePos = REST_STATE.jointOnePos;
        state.jointTwoPos = REST_STATE.jointTwoPos;
        INSTANCE = this;
    }

    // GETTERS

    public ClawState getState() {
        return state;
    }

    // SETTERS

    public void setState(ClawState targetState) {
        state.clawPos = targetState.clawPos;
        state.wristPos = targetState.wristPos;
        state.jointOnePos = targetState.jointOnePos;
        state.jointTwoPos = targetState.jointTwoPos;
    }

    public void openClaw() {
        state.clawPos = 0;
    }

    public void closeClaw() {
        state.clawPos = CLAW_CLOSE_POSITION;
    }

    public void setWrist(double pos) {
        state.wristPos = pos;
    }

    public void setJointOne(double pos){
        state.jointOnePos = pos;
    }

    public void setJointTwo(double pos){
        state.jointTwoPos = pos;
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
