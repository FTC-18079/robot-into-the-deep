package org.firstinspires.ftc.teamcode.util;

public interface SubsystemIF {
    public SubsystemIF initialize();
    public void configureHardware();
    public void onAutonomousInit();
    public void onTeleopInit();
}
