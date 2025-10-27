package org.firstinspires.ftc.teamcode.systems;

import android.util.Pair;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Localizer {

    private GoBildaPinpointDriver pinpoint;

    public static Pose2D pose = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS,0);
    private Pose2D truePose = new Pose2D(DistanceUnit.INCH,0,0, AngleUnit.RADIANS,0);

    public Pair<Double, Double> redGoalPoint = new Pair<>(72.0, -72.0), blueGoalPoint = new Pair<>(72.0, 72.0);

    public static GoBildaPinpointDriver.EncoderDirection yDirection, xDirection;


    public Localizer(HardwareMap hardwareMap, boolean resetPose) {
        if (!resetPose) {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setPosition(pose);
        } else {
            pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0,AngleUnit.RADIANS, 0));
        }

        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(xDirection, yDirection);

        pinpoint.setOffsets(-92.15, -111.625, DistanceUnit.MM);
    }

    public void update() {
        pinpoint.update();
        truePose = pinpoint.getPosition();
        pose = truePose;
    }

   public double angleToGoal(boolean red) {

        if (red) {
            double heading = truePose.getHeading(AngleUnit.RADIANS);

            double dx = redGoalPoint.first - truePose.getX(DistanceUnit.INCH);
            double dy = redGoalPoint.second - truePose.getY(DistanceUnit.INCH);

            double localizedDx = (dx * Math.cos(heading)) + (dy * Math.sin(heading));
            double localizedDY = -(dx * Math.sin(heading)) + (dy * Math.cos(heading));

            return (Math.toDegrees(Math.atan2(localizedDY, localizedDx))) % 360;
        } else {
            double heading = pose.getHeading(AngleUnit.RADIANS);

            double dx = blueGoalPoint.first - pose.getX(DistanceUnit.INCH);
            double dy = blueGoalPoint.second - pose.getY(DistanceUnit.INCH);

            double localizedDx = (dx * Math.cos(heading)) + (dy * Math.sin(heading));
            double localizedDY = -(dx * Math.sin(heading)) + (dy * Math.cos(heading));

            return (Math.toDegrees(Math.atan2(localizedDY, localizedDx)) + 360) % 360;
        }
   }

   public void setPose(Pose2D pose) {
        truePose = pose;
        pinpoint.setPosition(pose);
   }

    public Pose2D getTruePose() {
        return truePose;
    }
}
