package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.collector.CollectorConstants;

import java.util.List;

public class SampleDetection extends SubsystemBase {
    Limelight3A limelight;
    Telemetry telemetry;
    LLResult result;
    List<LLResultTypes.ColorResult> colorResults;

    private static SampleDetection INSTANCE = null;

    public static SampleDetection getInstance() {
        if (INSTANCE == null) INSTANCE = new SampleDetection();
        return INSTANCE;
    }

    public static void resetInstance() {
        INSTANCE.stop();
        INSTANCE = null;
    }

    private SampleDetection() {
        limelight = RobotMap.getInstance().LIMELIGHT;
        telemetry = RobotCore.getTelemetry();
        result = null;
        colorResults = null;

        setPipeline(0);
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
        if (result != null) {
            if (result.isValid()) {
                colorResults = result.getColorResults();
            }
            else colorResults = null;
        }
        else colorResults = null;
    }

    public double getSampleAngle() {
        if (colorResults == null) return CollectorConstants.PIVOT_PASSTHROUGH_POS;
        List<List<Double>> corners = colorResults.get(0).getTargetCorners();

        return 0;
    }

    @Override
    public void periodic() {
        updateResults();

        telemetry.addLine();
        telemetry.addData("Number of color results", colorResults.size());
        telemetry.addData("Number of corners", colorResults.size());
        telemetry.addData("LLResult", "tx: %d, ty: %d", result.getTx(), result.getTy());
    }
}
