package org.firstinspires.ftc.teamcode.util;

import com.arcrobotics.ftclib.command.SubsystemBase;

public abstract class SubsystemIF extends SubsystemBase {
    public SubsystemIF initialize() { return this; }
    public abstract void onAutonomousInit();
    public abstract void onTeleopInit();
}
