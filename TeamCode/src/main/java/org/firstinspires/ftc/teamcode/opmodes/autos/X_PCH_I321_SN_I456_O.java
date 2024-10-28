package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.ASCENT_PARKING_POSE;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.OBVZONE_PARKING_POSE;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.OBVZONE_STARTING_POSE;
import static org.firstinspires.ftc.teamcode.auto.AutoConstants.checkAlliance;
import static org.firstinspires.ftc.teamcode.util.EasyPathBuilder.constantHeadingPath;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.parkingPose;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.robotPose;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.auto.AutoConstants;
import org.firstinspires.ftc.teamcode.auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.chassis.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.collector.Collector;
import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.elevator.commands.ElevatorCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

@Disabled
@Autonomous(name = "X PCH I321 SN I456 O")
public class X_PCH_I321_SN_I456_O extends AutoTemplate {
    // Poses
    Pose scorePreloadPose = checkAlliance(new Pose(37, 72, Math.toRadians(0)));
    Pose sampleOnePose = checkAlliance(new Pose(65, 120, Math.toRadians(0)));
    Pose scoreOnePose = checkAlliance(new Pose(10.5, 120, 0));
    Pose sampleTwoPose = checkAlliance(new Pose(62, 130, 0));
    Pose scoreTwoPose = checkAlliance(new Pose(18, 130, 0));
    Pose sampleThreePose = checkAlliance(new Pose(62, 135.5, 0));
    Pose scoreThreePose = checkAlliance(new Pose(22, 135.5, 0));
    Pose parkPosition;

    // Paths
    Path scorePreloadPath;
    Path preloadToSampleOnePath;
    Path scoreOnePath;
    Path sampleTwoPath;
    Path scoreTwoPath;
    Path sampleThreePath;
    Path scoreThreePath;
    Path scoreToParkPath;

    @Override
    protected void setStartPose() {
        robotPose = checkAlliance(OBVZONE_STARTING_POSE);
        type = RobotCore.OpModeType.AUTO;
        parkPosition = (parkingPose == AutoConstants.ParkingPose.OBSERVATION_ZONE) ? OBVZONE_PARKING_POSE : ASCENT_PARKING_POSE;
        parkPosition = checkAlliance(parkPosition);
    }

    @Override
    public void buildPaths() {
        scorePreloadPath = constantHeadingPath(robotPose, scorePreloadPose, robotPose.getHeading());

        preloadToSampleOnePath = new Path(new BezierCurve(new Point(scorePreloadPose), new Point(17, 125, Point.CARTESIAN), new Point(70, 94, Point.CARTESIAN), new Point(sampleOnePose)));
        preloadToSampleOnePath.setConstantHeadingInterpolation(0);

        scoreOnePath = new Path(new BezierLine(new Point(sampleOnePose), new Point(scoreOnePose)));
        scoreOnePath.setConstantHeadingInterpolation(0);

        sampleTwoPath = new Path(new BezierCurve(new Point(scoreOnePose), new Point(64, 108, Point.CARTESIAN), new Point(sampleTwoPose)));
        sampleTwoPath.setConstantHeadingInterpolation(0);

        scoreTwoPath = new Path(new BezierLine(new Point(sampleTwoPose), new Point(scoreTwoPose)));
        scoreTwoPath.setConstantHeadingInterpolation(0);

        sampleThreePath = new Path(new BezierCurve(new Point(scoreTwoPose), new Point(60, 124, Point.CARTESIAN), new Point(sampleThreePose)));
        sampleThreePath.setConstantHeadingInterpolation(0);

        scoreThreePath = new Path(new BezierLine(new Point(sampleThreePose), new Point(scoreThreePose)));
        scoreThreePath.setConstantHeadingInterpolation(0);

        if (parkingPose == AutoConstants.ParkingPose.OBSERVATION_ZONE) {
            scoreToParkPath = new Path(new BezierCurve(new Point(scoreThreePose), new Point(50, 70, Point.CARTESIAN), new Point(parkPosition)));
            scoreToParkPath.setLinearHeadingInterpolation(scoreThreePose.getHeading(), parkPosition.getHeading());
        } else {
            scoreToParkPath = new Path(new BezierCurve(new Point(scoreThreePose), new Point(65, 125, Point.CARTESIAN), new Point(parkPosition)));
            scoreToParkPath.setLinearHeadingInterpolation(scoreThreePose.getHeading(), parkPosition.getHeading());
        }
    }

    @Override
    protected void initSequence() {
        Elevator.getInstance().closeClaw();
        Elevator.getInstance().closeDoor();
        Collector.getInstance().release();
    }

    @Override
    protected void rotatePoses() {

    }

    @Override
    protected Command makeAutoSequence() {
        return Commands.waitMillis(RobotGlobal.delayMs)
                .andThen(Commands.runOnce(Elevator.getInstance()::restBucket))
                .andThen(Commands.runOnce(() -> Elevator.getInstance().setScoreType(Elevator.ScoreType.SPECIMEN)))
                .andThen(Commands.runOnce(Elevator.getInstance()::toHigh))
                .andThen(new FollowPathCommand(scorePreloadPath))
                .andThen(Commands.waitMillis(200))
                .andThen(Commands.deferredProxy(() -> ElevatorCommands.SCORE_COMMAND))
                .andThen(new FollowPathCommand(preloadToSampleOnePath))
                .andThen(Commands.waitMillis(500))
                .andThen(new FollowPathCommand(scoreOnePath))
                .andThen(Commands.waitMillis(500))
                .andThen(new FollowPathCommand(scoreToParkPath));
    }
}
