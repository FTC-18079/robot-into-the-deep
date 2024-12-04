package org.firstinspires.ftc.teamcode.util.opmode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.util.hardware.AxonEncoder;

@TeleOp
public class AxonEncoderTest extends OpMode {
    private AxonEncoder encoder;

    @Override
    public void init() {
        encoder = new AxonEncoder(hardwareMap.get(AnalogInput.class, "pivotEncoder"));
    }

    @Override
    public void loop() {
        telemetry.addData("Encoder pos", encoder.getPosition());
        telemetry.addData("Manufacturer", encoder.getDevice().getManufacturer());
        telemetry.addData("Connection info", encoder.getDevice().getConnectionInfo());
        telemetry.update();
    }
}
