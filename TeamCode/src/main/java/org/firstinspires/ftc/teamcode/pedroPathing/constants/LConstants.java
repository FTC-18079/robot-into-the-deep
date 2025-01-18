package org.firstinspires.ftc.teamcode.pedroPathing.constants;

import com.pedropathing.localization.constants.OTOSConstants;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Localizer constants for Pedro Pathing
public class LConstants {
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
        OTOSConstants.linearScalar = 0.975433;
        OTOSConstants.angularScalar = 0.965;
    }
}
