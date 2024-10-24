package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.*;
import static org.firstinspires.ftc.teamcode.util.EasyPathBuilder.*;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.parkingPose;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.robotPose;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.collector.Collector;
import org.firstinspires.ftc.teamcode.collector.commands.CollectorCommands;
import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.elevator.commands.ElevatorCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.chassis.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

//@Photon
@Autonomous(name = "OZ 1+3 Alliance Samples")
public class OZ_1_3_Neutral extends AutoTemplate {
    // Poses
    Pose scoreBasketPose = checkAlliance(BASKET_SCORE_POSE);
    Pose scorePreloadPose = checkAlliance(new Pose(37, 72, Math.toRadians(0)));
    Pose sampleOnePose = checkAlliance(new Pose(32, 106, Math.toRadians(45)));
    Pose sampleOneControl = checkAlliance(new Pose(28, 98));
    Pose sampleTwoPose = checkAlliance(new Pose(45, 108.25, Math.toRadians(270)));
    Pose sampleTwoControl = checkAlliance(new Pose(28, 106));
    Pose sampleThreePose = checkAlliance(new Pose(45, 118.5, Math.toRadians(270)));
    Pose parkPosition;

    // Paths
    Path scorePreloadPath;
    Path preloadToCollectPath;
    Path collectOneToBasketPath;
    Path basketToCollectTwoPath;
    Path collectTwoToBasketPath;
    Path basketToCollectThreePath;
    Path collectThreeToBasketPath;
    Path basketToParkPath;
//    Path preloadToBasketPath = linearHeadingPath(scorePreloadPose, scoreBasketPose, scorePreloadPose.getHeading(), scoreBasketPose.getHeading(), 0.8, new Point(new Pose(23, 86)));

    @Override
    protected void setStartPose() {
        robotPose = checkAlliance(OBVZONE_STARTING_POSE);
        type = RobotCore.OpModeType.AUTO;
        parkPosition = (parkingPose == ParkingPose.OBSERVATION_ZONE) ? OBVZONE_PARKING_POSE : ASCENT_PARKING_POSE;
        parkPosition = checkAlliance(parkPosition);
    }

    @Override
    public void buildPaths() {
        scorePreloadPath = constantHeadingPath(robotPose, scorePreloadPose, robotPose.getHeading());

        preloadToCollectPath = new Path(new BezierCurve(new Point(scorePreloadPose), new Point(sampleOneControl), new Point(sampleOnePose)));
        preloadToCollectPath.setLinearHeadingInterpolation(scorePreloadPose.getHeading(), sampleOnePose.getHeading());

        collectOneToBasketPath = new Path(new BezierLine(new Point(sampleOnePose), new Point(scoreBasketPose)));
        collectOneToBasketPath.setLinearHeadingInterpolation(sampleOnePose.getHeading(), scoreBasketPose.getHeading());

        basketToCollectTwoPath = new Path(new BezierCurve(new Point(scoreBasketPose), new Point(sampleTwoControl), new Point(sampleTwoPose)));
        basketToCollectTwoPath.setLinearHeadingInterpolation(scoreBasketPose.getHeading(), sampleTwoPose.getHeading());

        collectTwoToBasketPath = new Path(new BezierLine(new Point(sampleTwoPose), new Point(scoreBasketPose)));
        collectTwoToBasketPath.setLinearHeadingInterpolation(sampleTwoPose.getHeading(), scoreBasketPose.getHeading());

        basketToCollectThreePath = new Path(new BezierLine(new Point(scoreBasketPose), new Point(sampleThreePose)));
        basketToCollectThreePath.setLinearHeadingInterpolation(scoreBasketPose.getHeading(), sampleThreePose.getHeading());

        collectThreeToBasketPath = new Path(new BezierLine(new Point(sampleThreePose), new Point(scoreBasketPose)));
        collectThreeToBasketPath.setLinearHeadingInterpolation(sampleThreePose.getHeading(), scoreBasketPose.getHeading());

        if (parkingPose == ParkingPose.OBSERVATION_ZONE) {
            basketToParkPath = new Path(new BezierCurve(new Point(scoreBasketPose), new Point(55, 27, Point.CARTESIAN), new Point(10, 24, Point.CARTESIAN)));
            basketToParkPath.setLinearHeadingInterpolation(scoreBasketPose.getHeading(), Math.toRadians(90));
        } else {
            basketToParkPath = new Path(new BezierLine(new Point(scoreBasketPose), new Point(parkPosition)));
            basketToParkPath.setLinearHeadingInterpolation(scoreBasketPose.getHeading(), parkPosition.getHeading());
        }
    }

    @Override
    protected void initSequence() {
        Elevator.getInstance().closeClaw();
        Elevator.getInstance().closeDoor();
        Collector.getInstance().release();
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
                .andThen(Commands.parallel(
                        Commands.runOnce(Collector.getInstance()::deployStow).andThen(CollectorCommands.TO_STOW.get()),
                        new FollowPathCommand(preloadToCollectPath)
                ))
                .andThen(Commands.runOnce(() -> Elevator.getInstance().setScoreType(Elevator.ScoreType.SAMPLE)))
                .andThen(CollectorCommands.TO_COLLECTING.get())
                .andThen(Commands.waitMillis(1000))
                .andThen(Commands.waitUntil(() -> Collector.getInstance().atSetPoint() && Collector.getInstance().getState() == Collector.CollectorState.COLLECTING))
                .andThen(Commands.waitMillis(500))
                .andThen(Commands.deferredProxy(() -> CollectorCommands.COLLECT_SEQUENCE))
                .andThen(Commands.waitMillis(1500))
                .andThen(Commands.runOnce(Elevator.getInstance()::toHigh))
                .andThen(new FollowPathCommand(collectOneToBasketPath))
                .andThen(Commands.deferredProxy(() -> ElevatorCommands.SCORE_COMMAND))
                .andThen(Commands.waitMillis(1000))
                .andThen(Commands.parallel(
                        Commands.deferredProxy(ElevatorCommands.RETRACT_COMMAND),
                        new FollowPathCommand(basketToCollectTwoPath)
                ));
    }
}
