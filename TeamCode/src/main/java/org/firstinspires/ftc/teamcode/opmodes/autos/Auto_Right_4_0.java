package org.firstinspires.ftc.teamcode.opmodes.autos;

import com.arcrobotics.ftclib.command.Command;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomous.AutoConstants;
import org.firstinspires.ftc.teamcode.autonomous.AutoTemplate;
import org.firstinspires.ftc.teamcode.claw.Claw;
import org.firstinspires.ftc.teamcode.claw.ClawConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;
import org.firstinspires.ftc.teamcode.util.RobotGlobal;
import org.firstinspires.ftc.teamcode.util.commands.Commands;

@Autonomous(name = "Right Side 4+0")
public class Auto_Right_4_0 extends AutoTemplate {
    // Poses
    private final Pose startingPose = new Pose(8, 64, Math.toRadians(180));
    private final Pose scorePreloadPose = AutoConstants.CHAMBER_RIGHT_SCORE_POSE;
    private final Pose behindOnePose = new Pose(62, 29, Math.toRadians(180));
    private final Pose pushOnePose = new Pose();
    private final Pose behindTwoPose = new Pose();
    private final Pose pushTwoPose = new Pose();

    // Paths
    private Path scorePreloadPath;
    private Path behindOnePath;
    private Path pushOnePath;
    private Path behindTwoPath;
    private Path pushTwoPath;
    private Path behindThreePath;
    private Path pushThreePath;

    @Override
    protected Pose getStartingPose() {
        return startingPose;
    }

    @Override
    protected void initSequence() {
        Claw.getInstance().setState(ClawConstants.REST_STATE);
        Claw.getInstance().periodic();
    }

    @Override
    protected void buildPaths() {
        scorePreloadPath = new Path(new BezierLine(new Point(startingPose), new Point(scorePreloadPose)));
        scorePreloadPath.setConstantHeadingInterpolation(scorePreloadPose.getHeading());
        scorePreloadPath.setPathEndTimeoutConstraint(200);
    }

    @Override
    protected Command makeAutoSequence() {
        return Commands.sequence(
                Commands.waitMillis(RobotGlobal.delayMs)
        );
    }
}
