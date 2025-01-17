package org.firstinspires.ftc.teamcode.newclaw;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hydra;
import org.firstinspires.ftc.teamcode.util.SubsystemIF;
import org.firstinspires.ftc.teamcode.util.hardware.AxonEncoder;

public class NewClaw extends SubsystemIF {
    private Telemetry telemetry;

    private Servo claw;
    private Servo joint;
    private Servo wrist;
    private CRServo turntable;
    private AxonEncoder turntableEncoder;

    private double wristPos;

    private static final NewClaw INSTANCE = new NewClaw();

    public static NewClaw getInstance() {
        return INSTANCE;
    }

    private NewClaw() {

    }

    // INIT

    @Override
    public void onAutonomousInit() {
        setWrist(0);
        telemetry = Hydra.getInstance().getTelemetry();
    }

    @Override
    public void onTeleopInit() {
        telemetry = Hydra.getInstance().getTelemetry();
    }

    // HARDWARE SETUP

    private void configureHardware() {

    }

    // SETTERS

    public void stowJoint() {

    }

    public void hoverJoint() {

    }

    public void deployJoint() {

    }

    public void grab() {

    }

    public void release() {

    }

    public void setWrist(double position) {
        // TODO: add wrapping lol
        wrist.setPosition(position);

    }

    public void setTurntable(double power) {
        turntable.setPower(power);
    }

    // PERIODIC

    @Override
    public void periodic() {
        super.periodic();
    }
}
