package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SuccessMotor  {
    private DcMotorEx motor;

    private double velocityThreshold = 10;
    private double lastVelocity = 0;

    public SuccessMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setVelocityThreshold(double velocityThreshold) {
        this.velocityThreshold = velocityThreshold;
    }

    public double getVelocityThreshold() {
        return velocityThreshold;
    }

    public boolean setVelocity(double velocity) {
        return setVelocityInternal(velocity);
    }

    private boolean setVelocityInternal(double velocity) {
        if (Math.abs(velocity - lastVelocity) > velocityThreshold) {
            lastVelocity = velocity;
            motor.setVelocity(velocity);
            return true;
        }
        return false;
    }

    public double getVelocity() {
        return motor.getVelocity();
    }

    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        motor.setZeroPowerBehavior(behavior);
    }

    public void setMode(DcMotor.RunMode mode) {
        motor.setMode(mode);
    }
}
