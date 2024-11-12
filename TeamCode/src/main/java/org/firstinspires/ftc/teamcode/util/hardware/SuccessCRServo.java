package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SuccessCRServo {
    private final CRServo servo;

    private double threshold = 0.005;
    private double lastPower = 0.0;

    public SuccessCRServo(CRServo servo) {
        this.servo = servo;
    }

    public void setPowerThreshold(double threshold) {
        this.threshold = threshold;
    }

    public boolean setPower(double power) {
        if (lastPower != 0 && power == 0) {
            servo.setPower(0);
            return true;
        }
        if (Math.abs(power - lastPower) > threshold) {
            lastPower = power;
            servo.setPower(power);
            return true;
        }
        return false;
    }

    public double getPower() {
        return servo.getPower();
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        servo.setDirection(direction);
    }
}
