package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Path;
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point;

import java.util.ArrayList;
import java.util.Arrays;

public class EasyPathBuilder {
    // Tangent heading paths
    public Path tangentHeadingPath(Pose startPose, Pose endPose, boolean reverse) {
        Path path = new Path(new BezierLine(new Point(startPose), new Point(endPose)));
        path.setTangentHeadingInterpolation();
        path.setReversed(reverse);
        return path;
    }

    public Path tangentHeadingPath(Pose startPose, Pose endPose, boolean reverse, Point... controlPoints) {
        ArrayList<Point> points = new ArrayList<>();

        points.add(new Point(startPose));
        points.add(new Point(endPose));
        points.addAll(Arrays.asList(controlPoints));

        Path path = new Path(new BezierCurve(points));
        path.setTangentHeadingInterpolation();
        path.setReversed(reverse);
        return path;
    }

    // Constant heading paths
    public static Path constantHeadingPath(Pose startPose, Pose endPose, double heading) {
        Path path = new Path(new BezierLine(new Point(startPose), new Point(endPose)));
        path.setConstantHeadingInterpolation(heading);
        return path;
    }

    public static Path constantHeadingPath(Pose startPose, Pose endPose, double heading, Point... controlPoints) {
        ArrayList<Point> points = new ArrayList<>();

        points.add(new Point(startPose));
        points.add(new Point(endPose));
        points.addAll(Arrays.asList(controlPoints));

        Path path = new Path(new BezierCurve(points));
        path.setConstantHeadingInterpolation(heading);
        return path;
    }

    // Linear heading interpolation paths
    public static Path linearHeadingPath(Pose startPose, Pose endPose, double startHeading, double endHeading) {
        return linearHeadingPath(startPose, endPose, startHeading, endHeading, 1.0);
    }

    public static Path linearHeadingPath(Pose startPose, Pose endPose, double startHeading, double endHeading, double endTime) {
        Path path = new Path(new BezierLine(new Point(startPose), new Point(endPose)));
        path.setLinearHeadingInterpolation(startHeading, endHeading, endTime);
        return path;
    }

    public static Path linearHeadingPath(Pose startPose, Pose endPose, double startHeading, double endHeading, Point... controlPoints) {
        return linearHeadingPath(startPose, endPose, startHeading, endHeading, 1.0, controlPoints);
    }

    public static Path linearHeadingPath(Pose startPose, Pose endPose, double startHeading, double endHeading, double endTime, Point... controlPoints) {
        ArrayList<Point> points = new ArrayList<>();

        points.add(new Point(startPose));
        points.add(new Point(endPose));
        points.addAll(Arrays.asList(controlPoints));

        Path path = new Path(new BezierCurve(points));
        path.setLinearHeadingInterpolation(startHeading, endHeading);
        return path;
    }
}
