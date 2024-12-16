package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AxonEncoder {
    private final AnalogInput encoder;
    private final boolean continuous;
    private double lastPosition;

    public AxonEncoder(AnalogInput encoder) {
        this(encoder, false);
    }

    public AxonEncoder(AnalogInput encoder, boolean continuous) {
        this.encoder = encoder;
        this.continuous = continuous;

        lastPosition = getPosition();
        encoder.resetDeviceConfigurationForOpMode();
    }

    /**
     * Get the position of the encoder
     * @return the position of the servo encoder in degrees
     */
    public double getPosition() {
        return encoder.getVoltage() / 3.3 * 360.0;
    }

    public AnalogInput getDevice() {
        return encoder;
    }
}
