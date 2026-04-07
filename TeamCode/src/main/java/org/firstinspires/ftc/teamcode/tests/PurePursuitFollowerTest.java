package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.LoopTimer;
import org.firstinspires.ftc.teamcode.systems.PinpointConstants;
import org.firstinspires.ftc.teamcode.systems.pureP.PurePursuitPath;
import org.firstinspires.ftc.teamcode.systems.pureP.PurePursuitPath.Point;
import org.firstinspires.ftc.teamcode.systems.pureP.PurePursuitSquidFollower;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDDrive;

@TeleOp
@Config
public class PurePursuitFollowerTest extends OpMode {


    public static double startHeadingRadians = 0;

    PurePursuitSquidFollower follower; SquIDDrive drive; boolean turretFlag = false;
    private GoBildaPinpointDriver pinpoint;
    public static GoBildaPinpointDriver.EncoderDirection yDirection, xDirection;

    public static double p1X = -14, p1Y = -25, p1H = 0, p2X = -14, p2Y = -25, p2H = 90;

    public Point[] path = {new Point(0, 0,0), new Point( p1X, p1Y, Math.toRadians(p1H)),
            new Point(p2X, p2Y, Math.toRadians(p2H))};

    public static double lookAheadRad = 15;
    public int recoverySegments = 2;

    PurePursuitPath purePursuitPath = new PurePursuitPath(path, lookAheadRad, recoverySegments);

    public static PurePursuitPath.HeadingMode headingMode = PurePursuitPath.HeadingMode.INDEPENDENT;

    private Intake intake;

    @Override
    public void init() {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(PinpointConstants.xDirection, PinpointConstants.yDirection);

        pinpoint.setOffsets(PinpointConstants.xOffset, PinpointConstants.yOffset, DistanceUnit.MM);

        drive = new SquIDDrive(hardwareMap, hardwareMap.voltageSensor.iterator().next());
        follower = new PurePursuitSquidFollower(drive, pinpoint);

        follower.setCurrentPose(0, 0, startHeadingRadians);

        purePursuitPath.setHeadingMode(headingMode);
        follower.setCurrentPath(purePursuitPath);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        intake = new Intake(hardwareMap);

        intake.setIntake(1);
        intake.setTransfer(false);

    }

    boolean pinpointReady = false;

    @Override
    public void init_loop() {
        if (!pinpointReady) {
            pinpoint.resetPosAndIMU();

            pinpoint.update();
        }
        if (pinpoint.getLoopTime() > 0) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, startHeadingRadians));
            if (pinpointReady) {
                pinpoint.update();
            }
            pinpointReady = true;
        }
    }

    LoopTimer timer = new LoopTimer();

    @Override
    public void loop() {
        timer.update();
        follower.read();
        follower.update();

        telemetry.addData("tx", follower.getTargetPose().getX());
        telemetry.addData("ty", follower.getTargetPose().getY());
        telemetry.addData("tz", Math.toDegrees(follower.getTargetPose().getZ()));

        telemetry.addData("cx", follower.getCurrentPose().getX());
        telemetry.addData("cy", follower.getCurrentPose().getY());
        telemetry.addData("cz", Math.toDegrees(follower.getCurrentPose().getZ()));


        telemetry.addData("Loop ms", timer.getLoopPeriodMs());
        telemetry.addData("Loop Hz", timer.getLoopHz());
        telemetry.addData("Loop slow?", timer.isSlowerThan(20));
        telemetry.update();
    }
}
