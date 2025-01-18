package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.localization.Localizers;
import com.pedropathing.util.CustomFilteredPIDFCoefficients;
import com.pedropathing.util.CustomPIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

// Follower constants for Pedro Pathing
public class FConstants {
    public static final String MOTOR_FL_NAME = "leftFront";
    public static final String MOTOR_FR_NAME = "rightFront";
    public static final String MOTOR_BL_NAME = "leftBack";
    public static final String MOTOR_BR_NAME = "rightBack";

    static {
    // General
        // Whether or not to brake in teleop
        FollowerConstants.useBrakeModeInTeleOp = true;
        // Threshold to cache motor powers
        FollowerConstants.motorCachingThreshold = 0.01;

        // Localizer type
        FollowerConstants.localizers = Localizers.OTOS;
        // Motor names
        FollowerConstants.leftFrontMotorName = MOTOR_FL_NAME;
        FollowerConstants.rightFrontMotorName = MOTOR_FR_NAME;
        FollowerConstants.leftRearMotorName = MOTOR_BL_NAME;
        FollowerConstants.rightRearMotorName = MOTOR_BR_NAME;
        // Motor directions
        FollowerConstants.leftFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.rightFrontMotorDirection = DcMotorSimple.Direction.FORWARD;
        FollowerConstants.leftRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        FollowerConstants.rightRearMotorDirection = DcMotorSimple.Direction.REVERSE;
        // Robot mass in kg
        FollowerConstants.mass = 15.422; // 34 lbs in freedom units
        // Max velocities
        FollowerConstants.xMovement = 76.205;
        FollowerConstants.yMovement = 51.4708;
        // Robot deceleration
        FollowerConstants.forwardZeroPowerAcceleration = -47.601;
        FollowerConstants.lateralZeroPowerAcceleration = -114.94575;
        // Zero power multiplier
        FollowerConstants.zeroPowerAccelerationMultiplier = 3.25;
        // Whether or not to use dual PID
        FollowerConstants.useSecondaryDrivePID = false;
        FollowerConstants.useSecondaryTranslationalPID = false;
        FollowerConstants.useSecondaryHeadingPID = false;
        // Primary PID coefficients
        FollowerConstants.drivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.007, 0, 0.00011, 0.6, 0);
        FollowerConstants.translationalPIDFCoefficients = new CustomPIDFCoefficients(0.7, 0, 0.06, 0);
        FollowerConstants.headingPIDFCoefficients = new CustomPIDFCoefficients(3.0, 0, 0.15, 0);
        // Secondary PID coefficients
        FollowerConstants.secondaryDrivePIDFCoefficients = new CustomFilteredPIDFCoefficients(0.010, 0, 0.00004, 0.6, 0);
        FollowerConstants.secondaryTranslationalPIDFCoefficients = new CustomPIDFCoefficients(0.3, 0, 0.01, 0);
        FollowerConstants.secondaryHeadingPIDFCoefficients = new CustomPIDFCoefficients(3.0, 0, 0.15, 0);
        // Centripetal force correction
        FollowerConstants.centripetalScaling = 0.0007;
    }
}
