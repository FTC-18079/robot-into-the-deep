package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.collector.CollectorConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.ArrayList;
import java.util.List;

public class LLVision extends SubsystemBase {
    Limelight3A limelight;
    Telemetry telemetry;
    LLResult result;
    List<LLResultTypes.ColorResult> colorResults;

    private static LLVision INSTANCE = null;

    public static LLVision getInstance() {
        if (INSTANCE == null) INSTANCE = new LLVision();
        return INSTANCE;
    }

    public static void resetInstance() {
        INSTANCE.stop();
        INSTANCE = null;
    }

    private LLVision() {
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
        ArrayList<Point> cornerList = new ArrayList<Point>();

        for (int i = 0; i < corners.size(); i ++) {
            cornerList.add(new Point(corners.get(i).get(0), corners.get(i).get(1), Point.CARTESIAN));
        }

        Point cornerOne = cornerList.get(getFarthestPointIndex(cornerList));

        return 0;
    }

    private int getFarthestPointIndex(List<Point> list) {
        double max = 0;
        int index = 0;

        for (int i = 0; i < list.size(); i++) {
            double radius = list.get(i).getR();
            if (radius > max) {
                max = radius;
                index = i;
            }
        }

        return index;
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
