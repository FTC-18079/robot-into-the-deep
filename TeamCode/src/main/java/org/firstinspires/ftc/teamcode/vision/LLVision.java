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
    LLResult lastResult;

    private static LLVision INSTANCE = null;

    public static LLVision getInstance() {
        if (INSTANCE == null) INSTANCE = new LLVision(null);
        return INSTANCE;
    }

    public static void resetInstance() {
        INSTANCE.stop();
        INSTANCE = null;
    }

    public LLVision(Telemetry telemetry) {
        limelight = RobotMap.getInstance().LIMELIGHT;
        this.telemetry = telemetry;
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

    @Override
    public void periodic() {
        updateResults();

        telemetry.addLine();
        telemetry.addData("same?", result.equals(lastResult));
        telemetry.addData("angle", getSampleAngle());
        telemetry.addData("time", limelight.getTimeSinceLastUpdate());
        telemetry.addData("Running?", limelight.isRunning());
        telemetry.addData("Connected?", limelight.isConnected());
        telemetry.addData("tx", result.getTx());
        telemetry.addData("ty", result.getTy());
        telemetry.addData("LL CPU", limelight.getStatus().getCpu());
        telemetry.addData("Valid result", result.isValid());

        lastResult = limelight.getLatestResult();
    }
}
