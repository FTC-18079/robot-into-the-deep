package org.firstinspires.ftc.teamcode.vision;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions;

import java.util.List;

// velo / (buffer * max) (this is for later lol)
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
        if (INSTANCE != null) INSTANCE.stop();
        INSTANCE = null;
    }

    private LLVision() {
        limelight = RobotMap.getInstance().LIMELIGHT;
        telemetry = RobotCore.getTelemetry();

        setPipeline(0);
        start();
    }

    // LIMELIGHT MANAGEMENT

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

    public void setYellow() {
        limelight.pipelineSwitch(0);
    }

    public void setRed() {
        limelight.pipelineSwitch(1);
    }

    public void setBlue() {
        limelight.pipelineSwitch(2);
    }

    /**
     * Updates limelight detection results
     */
    public void updateResults() {
        result = limelight.getLatestResult();
        colorResults = result.getColorResults();
    }

    // GETTERS

    /**
     * Gets the angle of the sample in degrees
     * @return The angle of the detected sample
     */
    public double getSampleAngle() {
        if (colorResults.isEmpty()) return 90.0;

        // Get sample corners
        List<List<Double>> corners = colorResults.get(0).getTargetCorners();

        if (corners.size() < 4) return 90;

        // Get corner positions
        Pose corner0 = new Pose(corners.get(0).get(0), corners.get(0).get(1));
        Pose corner1 = new Pose(corners.get(1).get(0), corners.get(1).get(1));
        Pose corner2 = new Pose(corners.get(2).get(0), corners.get(2).get(1));
        Pose corner3 = new Pose(corners.get(3).get(0), corners.get(3).get(1));

        // Retrieve rectangle side lengths
        double[] distances = {
                MathFunctions.distance(corner0, corner1),
                MathFunctions.distance(corner1, corner2),
                MathFunctions.distance(corner2, corner3),
                MathFunctions.distance(corner3, corner0)
        };

        double max = 0;
        for (int i = 0; i < 4; i++) {
            if (max < distances[i]) {
                max = distances[i];
            }
        }

        // Get angle based off the longest side length
        Pose pose1;
        Pose pose2;
        if (max == distances[0]) {
            pose1 = corner0;
            pose2 = corner1;
        } else if (max == distances[1]) {
            pose1 = corner1;
            pose2 = corner2;
        } else if (max == distances[2]) {
            pose1 = corner2;
            pose2 = corner3;
        } else {
            pose1 = corner3;
            pose2 = corner0;
        }

        double angle = Math.atan((pose1.getY() - pose2.getY()) / (pose1.getX() - pose2.getX()));

        return Math.toDegrees(angle);
    }

    /**
     * Gets the servo position based off the sample angle
     * @return A servo position ranging from 0.0 to 1.0
     */
    public double getServoPos() {
        // Convert the angle to a servo position
        return 1 - (getSampleAngle() / 180 + 0.5);
    }

    public LLResult getResult() {
        return result;
    }

    public boolean colorResultExists() {
        return !colorResults.isEmpty();
    }

    public boolean isRunning() {
        return limelight.isRunning();
    }

    @Override
    public void periodic() {
        updateResults();
    }
}
