package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SuccessMotor  {
    private DcMotorEx motor;

    private double threshold = 0.005;
    private double lastPower = 0;

    public SuccessMotor(DcMotorEx motor) {
        this.motor = motor;
    }

    public void setThreshold(double threshold) {
        this.threshold = threshold;
    }

    public double getThreshold() {
        return threshold;
    }

    public boolean setPower(double velocity) {
        return setPowerInternal(velocity);
    }

    private boolean setPowerInternal(double power) {
        if (Math.abs(power - lastPower) > threshold) {
            lastPower = power;
            motor.setPower(power);
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
