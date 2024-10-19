package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotMap;

public class LLVision extends SubsystemBase {
    Limelight3A limelight;
    Telemetry telemetry;
    LLResult result;
    double[] pythonOutput;

    private static LLVision INSTANCE = null;

    public static LLVision getInstance() {
        if (INSTANCE == null) INSTANCE = new LLVision();
        return INSTANCE;
    }

    public static void resetInstance() {
        if (INSTANCE != null) INSTANCE.stop();
        INSTANCE = null;
    }

    private LLVision() {
        limelight = RobotMap.getInstance().LIMELIGHT;
//        telemetry = RobotCore.getTelemetry();

        setPipeline(4);
        start();
    }

    public void start() {
        limelight.start();
    }

    public void stop() {
        limelight.stop();
    }

    /**
     * Switches the limelight's active pipeline to the index
     *
     * @param index The index of the pipeline. Yellow = 0, Red = 1, Blue = 2
     */
    public void setPipeline(int index) {
        limelight.pipelineSwitch(index);
    }

    public void updateResults() {
        result = limelight.getLatestResult();
        pythonOutput = result.getPythonOutput();
    }

    public void setColorRange(double[] range) {
        limelight.updatePythonInputs(range);
    }

    public double getSampleAngle() {
        return pythonOutput[0];
    }

    public double getServoPos() {
        return 1 - (getSampleAngle() / 180.0);
    }

    public boolean isRunning() {
        return limelight.isRunning();
    }

    @Override
    public void periodic() {
        updateResults();
    }
}
