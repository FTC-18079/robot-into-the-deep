package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.Servo;

public class SuccessServo {
    private Servo servo;

    private double posThreshold = 0.02;
    private double lastPos = 0;

    public SuccessServo(Servo servo) {
        this.servo = servo;
    }

    public void setPosThreshold(double threshold) {
        posThreshold = threshold;
    }

    public boolean setPosition(double position) {
        return setPositionInternal(position);
    }

    private boolean setPositionInternal(double position) {
        if (Math.abs(position - lastPos) > posThreshold) {
            lastPos = position;
            servo.setPosition(position);
            return true;
        }
        return false;
    }

    public double getPosition() {
        return servo.getPosition();
    }
}
