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
import org.firstinspires.ftc.teamcode.util.SubsystemIF;

import java.util.ArrayList;
import java.util.List;

public class LLVision extends SubsystemBase implements SubsystemIF {
    Limelight3A limelight;
    Telemetry telemetry;
    LLResult result;
    List<LLResultTypes.ColorResult> colorResults;

    SampleColor targetColor;
    double clawOverride = -1;

    private static final LLVision INSTANCE = new LLVision();

    public static LLVision getInstance() {
        return INSTANCE;
    }

    public enum SampleColor {
        YELLOW(0),
        RED(1),
        BLUE(2);

        public final int index;

        private SampleColor(int index) {
            this.index = index;
        }
    }

    private LLVision() {
        limelight = RobotMap.getInstance().LIMELIGHT;
        telemetry = RobotCore.getTelemetry();

        targetColor = SampleColor.YELLOW;
        setPipeline();
        start();
    }

    // INITIALIZE

    @Override
    public void onTeleopInit() {
        limelight = RobotMap.getInstance().LIMELIGHT;
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
     * Switches the limelight's pipeline
     *
     */
    private void setPipeline() {
        limelight.pipelineSwitch(targetColor.index);
    }

    public void setYellow() {
        targetColor = SampleColor.YELLOW;
        setPipeline();
    }

    public void setRed() {
        targetColor = SampleColor.RED;
        setPipeline();
    }

    public void setBlue() {
        targetColor = SampleColor.BLUE;
        setPipeline();
    }

    /**
     * Updates limelight detection results
     */
    public void updateResults() {
        result = limelight.getLatestResult();
        if (result == null) {
            colorResults = new ArrayList<>();
        } else {
            colorResults = result.getColorResults();
        }
    }

    public void setClawOverride(double pos) {
        clawOverride = pos;
    }

    public void disableClawOverride() {
        setClawOverride(-1);
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

        return Math.toDegrees(MathFunctions.normalizeAngle(angle));
    }

    public double getSampleTy() {
        if (colorResults.isEmpty()) return 0.0;
        return colorResults.get(0).getTargetYDegrees();
    }

    /**
     * Gets the servo position based off the sample angle
     * @return A servo position ranging from 0.0 to 1.0
     */
    public double getServoPos() {
        if (clawOverride == -1) {
            return VisionConstants.getNearestClawAngle(getSampleAngle());
        }
        return clawOverride;
    }

    public LLResult getResult() {
        return result;
    }

    public SampleColor getTargetColor() {
        return targetColor;
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

        telemetry.addLine();
        telemetry.addData("Target color", targetColor);
        telemetry.addData("Sample ty", getSampleTy());
        telemetry.addData("Claw pos", getServoPos());
        telemetry.addData("Sample angle", getSampleAngle());
    }
}
