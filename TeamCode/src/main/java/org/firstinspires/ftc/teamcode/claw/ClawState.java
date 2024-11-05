package org.firstinspires.ftc.teamcode.claw;

import androidx.annotation.NonNull;

public class ClawState {
    public double clawPos;
    public double wristPos;
    public double jointPos;

    public ClawState() {
        this(0, 0 ,0);
    }

    public ClawState(double clawPos, double wristPos, double jointPos) {
        this.clawPos = clawPos;
        this.wristPos = wristPos;
        this.jointPos = jointPos;
    }

    @NonNull
    @Override
    public String toString() {
        return "Claw State(Claw: " + clawPos + ", Wrist: " + wristPos + ", Joint: " + jointPos + ")";
    }
}
