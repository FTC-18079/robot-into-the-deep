package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.constants.OTOSConstants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Localizer constants for Pedro Pathing
public class LConstants {
    // Average of one 100 inch test, two 60 inch tests, one 48 inch test
    private static final double LINEAR_SCALAR = (1.0230442666666666 + 1.0063494266413662) / 2;
    // Average of one 1 rotation test, two 3 rotation tests, and one 5 rotation test
    private static final double ANGULAR_SCALAR = (0.9749 + 0.9847 + 0.9842 + 0.9818) / 4;

    static {
        // Whether or not to use the "corrected" otos driver
        OTOSConstants.useCorrectedOTOSClass = false;
        // Device name
        OTOSConstants.hardwareMapName = "otos";
        // Localizer units
        OTOSConstants.linearUnit = DistanceUnit.INCH;
        OTOSConstants.angleUnit = AngleUnit.RADIANS;
        // OTOS position on the robot
        OTOSConstants.offset = new SparkFunOTOS.Pose2D(1.28100393701, 0, 0);
        // Scalars for the sensor
        OTOSConstants.linearScalar = LINEAR_SCALAR;
        OTOSConstants.angularScalar = ANGULAR_SCALAR;
    }
}
