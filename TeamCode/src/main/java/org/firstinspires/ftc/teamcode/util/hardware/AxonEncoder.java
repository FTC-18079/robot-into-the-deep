package org.firstinspires.ftc.teamcode.util.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AxonEncoder {
    private final AnalogInput encoder;

    public AxonEncoder(AnalogInput encoder) {
        this.encoder = encoder;
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
