package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;


@Config
public class PinpointConstants {

    public static GoBildaPinpointDriver.EncoderDirection yDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED,
            xDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public static double xOffset = 118, yOffset = 20;

    public static SparkFunOTOS.Pose2D globalPose = new SparkFunOTOS.Pose2D(0, 0, 0);
    
    public static void initializePinpoint(GoBildaPinpointDriver pinpoint) {
        pinpoint.setEncoderDirections(xDirection, yDirection);
        pinpoint.setOffsets(xOffset, yOffset, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, globalPose.x, globalPose.y, AngleUnit.RADIANS, globalPose.h));
    }
}
