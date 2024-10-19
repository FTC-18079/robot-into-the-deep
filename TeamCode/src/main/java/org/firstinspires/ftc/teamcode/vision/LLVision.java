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
        colorResults = result.getColorResults();
//        if (result != null) {
//            if (result.isValid()) {
//                colorResults = result.getColorResults();
//            }
//            else colorResults = null;
//        }
//        else colorResults = null;
    }

    public double getSampleAngle() {
        if (colorResults.isEmpty()) return CollectorConstants.PIVOT_PASSTHROUGH_POS;
        List<List<Double>> corners = colorResults.get(0).getTargetCorners();

        Point corner0 = new Point(corners.get(0).get(0), corners.get(0).get(1), Point.CARTESIAN);
        Point corner1 = new Point(corners.get(1).get(0), corners.get(1).get(1), Point.CARTESIAN);

        double angle = Math.toDegrees(Math.atan2(corner1.getY() - corner0.getY(), corner1.getX() - corner0.getX()));

        angle = (angle + 180) / 360.0;

        return angle;
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
        for (LLResultTypes.ColorResult cr : colorResults) {
            telemetry.addData("Corner num", cr.getTargetCorners().size());
            telemetry.addData("Color", "X: %.2f, Y: %.2f", cr.getTargetXDegrees(), cr.getTargetYDegrees());
        }
        telemetry.addData("angle", getSampleAngle());
        telemetry.addData("tx", result.getTx());
        telemetry.addData("ty", result.getTy());
    }
}
