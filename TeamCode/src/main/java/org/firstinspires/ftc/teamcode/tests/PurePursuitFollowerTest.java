package org.firstinspires.ftc.teamcode.tests;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.pedropathing.geometry.Pose;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.systems.LoopTimer;
import org.firstinspires.ftc.teamcode.systems.pureP.PurePursuitPath;
import org.firstinspires.ftc.teamcode.systems.pureP.PurePursuitPath.Point;
import org.firstinspires.ftc.teamcode.systems.pureP.PurePursuitSquidDrive;
import org.firstinspires.ftc.teamcode.systems.pureP.PurePursuitSquidFollower;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDDrive;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDFollower;

@TeleOp
@Config
public class PurePursuitFollowerTest extends OpMode {

    PurePursuitSquidFollower follower; PurePursuitSquidDrive drive; boolean turretFlag = false;
    private GoBildaPinpointDriver pinpoint;
    public static GoBildaPinpointDriver.EncoderDirection yDirection, xDirection;

    public Point[] path = {new Point(0, 0,0), new Point( 30, 30, 0),
            new Point(60, 0, 0), new Point(30, -20, 0), new Point(0, -20, 0)};

    public static double lookAheadRad = 10;
    public int recoverySegments = 2;

    PurePursuitPath purePursuitPath = new PurePursuitPath(path, lookAheadRad, recoverySegments);

    public static PurePursuitPath.HeadingMode headingMode = PurePursuitPath.HeadingMode.TANGENT;

    @Override
    public void init() {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(xDirection, yDirection);

        pinpoint.setOffsets(-92.15, -111.625, DistanceUnit.MM);

        drive = new PurePursuitSquidDrive(hardwareMap, hardwareMap.voltageSensor.iterator().next());
        follower = new PurePursuitSquidFollower(drive, pinpoint);

        follower.setCurrentPose(0, 0, 0);

        purePursuitPath.setHeadingMode(headingMode);
        follower.setCurrentPath(purePursuitPath);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

    }

    boolean pinpointReady = false;

    @Override
    public void init_loop() {
        if (!pinpointReady) {
            pinpoint.resetPosAndIMU();

            pinpoint.update();
        }
        if (pinpoint.getLoopTime() > 0) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));
            follower.setCurrentPose(0, 0, 0);
            if (pinpointReady) {
                pinpoint.update();
            }
            pinpointReady = true;
        }
        SparkFunOTOS.Pose2D cur = follower.getCurrentPose();

        telemetry.addData("Current X", cur.x);
        telemetry.addData("Current Y", cur.y);
        telemetry.addData("Current H", cur.h);
        telemetry.update();
    }

    LoopTimer timer = new LoopTimer();

    @Override
    public void loop() {
        timer.update();
        follower.read();
        follower.update();
        telemetry.addData("Loop ms", timer.getLoopPeriodMs());
        telemetry.addData("Loop Hz", timer.getLoopHz());
        telemetry.addData("Loop slow?", timer.isSlowerThan(20));
        telemetry.update();
    }
}
