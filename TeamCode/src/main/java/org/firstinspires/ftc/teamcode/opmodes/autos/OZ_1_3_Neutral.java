package org.firstinspires.ftc.teamcode.opmodes.autos;

import static org.firstinspires.ftc.teamcode.auto.AutoConstants.*;
import static org.firstinspires.ftc.teamcode.util.EasyPathBuilder.*;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.parkingPose;
import static org.firstinspires.ftc.teamcode.util.RobotGlobal.robotPose;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.outoftheboxrobotics.photoncore.Photon;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotCore;
import org.firstinspires.ftc.teamcode.auto.AutoTemplate;
import org.firstinspires.ftc.teamcode.chassis.Chassis;
import org.firstinspires.ftc.teamcode.collector.Collector;
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
    Pose scoreBasketPose = checkAlliance(BASKET_SCORE_POSE);
    Pose parkPosition;

    // Paths
    Path scorePreloadPath;
    Path preloadToBasketPath = new Path(new BezierCurve(new Point(scorePreloadPose), new Point(scoreBasketPose)));
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
                .andThen(Commands.deferredProxy(() -> ElevatorCommands.SCORE_COMMAND));
    }
}
