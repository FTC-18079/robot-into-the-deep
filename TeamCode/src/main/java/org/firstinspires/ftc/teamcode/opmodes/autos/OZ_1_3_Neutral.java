package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.*;
import static org.firstinspires.ftc.teamcode.util.EasyPathBuilder.*;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.parkingPose;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.robotPose;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.collector.Collector;
import org.firstinspires.ftc.teamcode.collector.commands.CollectorCommands;
import org.firstinspires.ftc.teamcode.elevator.Elevator;
import org.firstinspires.ftc.teamcode.elevator.commands.ElevatorCommands;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.chassis.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

//@Photon
@Autonomous(name = "OZ 1+3 Alliance Samples")
public class OZ_1_3_Neutral extends AutoTemplate {
    // Poses
    Pose scorePreloadPose = checkAlliance(new Pose(37, 72, Math.toRadians(0)));
    Pose sampleOnePose = checkAlliance(new Pose(20, 120, Math.toRadians(180)));
    Pose sampleOneControl = checkAlliance(new Pose(22, 98));
    Pose scoreBasketPose = checkAlliance(BASKET_SCORE_POSE);
    Pose parkPosition;

    // Paths
    Path scorePreloadPath;
    Path preloadToCollectPath;
//    Path preloadToBasketPath = linearHeadingPath(scorePreloadPose, scoreBasketPose, scorePreloadPose.getHeading(), scoreBasketPose.getHeading(), 0.8, new Point(new Pose(23, 86)));

    @Override
    protected void setStartPose() {
        robotPose = checkAlliance(OBVZONE_STARTING_POSE);
        type = RobotCore.OpModeType.AUTO;
        parkPosition = (parkingPose == ParkingPose.OBSERVATION_ZONE) ? OBVZONE_PARKING_POSE : SUBMERSIBLE_PARKING_POSE;
        parkPosition = checkAlliance(parkPosition);
    }

    @Override
    public void buildPaths() {
        scorePreloadPath = constantHeadingPath(robotPose, scorePreloadPose, robotPose.getHeading());

        preloadToCollectPath = new Path(new BezierCurve(new Point(scorePreloadPose), new Point(sampleOneControl), new Point(sampleOnePose)));
        preloadToCollectPath.setLinearHeadingInterpolation(scorePreloadPose.getHeading(), sampleOnePose.getHeading());
    }

    @Override
    protected void initSequence() {
        Elevator.getInstance().closeClaw();
        Elevator.getInstance().closeDoor();
        Elevator.getInstance().restBucket();
        Collector.getInstance().release();
    }

    @Override
    protected Command makeAutoSequence() {
        return new WaitCommand(RobotGlobal.delayMs)
                .andThen(Commands.runOnce(() -> Elevator.getInstance().setScoreType(Elevator.ScoreType.SPECIMEN)))
                .andThen(Commands.runOnce(Elevator.getInstance()::toHigh))
                .andThen(new FollowPathCommand(scorePreloadPath))
                .andThen(new WaitCommand(200))
                .andThen(Commands.deferredProxy(() -> ElevatorCommands.SCORE_COMMAND))
                .andThen(Commands.parallel(
                        Commands.runOnce(Collector.getInstance()::deployStow).andThen(CollectorCommands.TO_STOW.get()),
                        new FollowPathCommand(preloadToCollectPath)
                ))
                .andThen(CollectorCommands.TO_COLLECTING.get())
                .andThen(Commands.waitMillis(200))
                .andThen(new WaitUntilCommand(() -> Collector.getInstance().atSetPoint() && Collector.getInstance().getState() == Collector.CollectorState.COLLECTING))
                .andThen(Commands.deferredProxy(() -> CollectorCommands.COLLECT_SEQUENCE));
    }
}
