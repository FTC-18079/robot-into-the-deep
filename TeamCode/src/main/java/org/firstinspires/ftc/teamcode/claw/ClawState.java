package org.firstinspires.ftc.teamcode.claw;

import androidx.annotation.NonNull;

public class ClawState {
    public double clawPos;
    public double wristPos;
    public double jointOnePos;
    public double jointTwoPos;

    public ClawState() {
        this(0, 0 ,0, 0);
    }

    public ClawState(double clawPos, double wristPos, double jointOnePos, double jointTwoPos) {
        this.clawPos = clawPos;
        this.wristPos = wristPos;
        this.jointOnePos = jointOnePos;
        this.jointTwoPos = jointTwoPos;
    }

    @NonNull
    @Override
    public String toString() {
        return "Claw State(Claw: " + clawPos + ", Wrist: " + wristPos + ", Joint One: " + jointOnePos + ", Joint Two: " + jointTwoPos +")";
    }
}
