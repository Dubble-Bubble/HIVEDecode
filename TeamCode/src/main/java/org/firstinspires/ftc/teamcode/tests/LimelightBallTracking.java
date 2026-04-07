package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.PinpointConstants;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDDrive;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDFollower;

import java.util.List;

@TeleOp
@Config
public class LimelightBallTracking extends OpMode {

    public static  double CAMERA_PITCH_DEG = 15, CAMERA_HEIGHT_IN = 8.5, CAMERA_FORWARD_OFFSET = 9;
    Limelight3A limelight;

    SquIDDrive drive; SquIDFollower follower;

    private GoBildaPinpointDriver pinpoint;
    private Intake intake;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(2);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");


        intake = new Intake(hardwareMap);

        drive = new SquIDDrive(hardwareMap, hardwareMap.voltageSensor.iterator().next());

        PinpointConstants.initializePinpoint(pinpoint);

        follower = new SquIDFollower(drive, pinpoint);

        intake.setIntake(1);
    }

    public static double threshold = 8.0;

    private double targetPoseY = 0;

    boolean pinpointReady = false;

    @Override
    public void init_loop() {
        LLResult result = limelight.getLatestResult();
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        for (LLResultTypes.DetectorResult detection : detections) {
            String className = detection.getClassName(); // What was detected
            double x = detection.getTargetXDegrees(); // Where it is (left-right)
            double y = detection.getTargetYDegrees(); // Where it is (up-down)
            telemetry.addData(className, "at (" + x + ", " + y + ") degrees");
        }

        double[] clumpCenter = findLargestClumpFieldCoords(detections, new SparkFunOTOS.Pose2D(56, 9, Math.toRadians(169)),threshold);

        telemetry.addData("largest cluster tx and ty: ", "(" + clumpCenter[0] + ", " + clumpCenter[1] + ")");
        targetPoseY = clumpCenter[1];
        telemetry.update();

        if (!pinpointReady) {
            pinpoint.resetPosAndIMU();

            pinpoint.update();
        }
        if (pinpoint.getLoopTime() > 0) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 56, 9, AngleUnit.RADIANS, Math.toRadians(169)));
            follower.setCurrentPose(56, 9, Math.toRadians(169));
            follower.setTargetPose(new SparkFunOTOS.Pose2D(56, 9, Math.toRadians(169)));

            if (pinpointReady) {
                pinpoint.update();
            }
            pinpointReady = true;
        }
    }

    @Override
    public void loop() {
        follower.setTargetPose(new SparkFunOTOS.Pose2D(9, targetPoseY, Math.toRadians(180)));
        follower.read();
        follower.update();
    }


    private static double[] findLargestClumpCenter(
            List<LLResultTypes.DetectorResult> detections, double threshold) {

        int n = detections.size();
        int[] clusterIds = new int[n];
        for (int i = 0; i < n; i++) clusterIds[i] = -1;

        int nextClusterId = 0;

        for (int i = 0; i < n; i++) {
            if (clusterIds[i] != -1) continue;

            double bestDist    = Double.MAX_VALUE;
            int    bestCluster = -1;

            for (int j = 0; j < i; j++) {
                double dist = angularDistance(detections.get(i), detections.get(j));
                if (dist < threshold && dist < bestDist) {
                    bestDist    = dist;
                    bestCluster = clusterIds[j];
                }
            }

            clusterIds[i] = (bestCluster != -1) ? bestCluster : nextClusterId++;
        }

        int[] clusterSizes = new int[nextClusterId];
        for (int id : clusterIds) clusterSizes[id]++;

        int largestId = 0;
        for (int i = 1; i < clusterSizes.length; i++) {
            if (clusterSizes[i] > clusterSizes[largestId]) largestId = i;
        }

        double sumTx = 0, sumTy = 0;
        int count = 0;
        for (int i = 0; i < n; i++) {
            if (clusterIds[i] == largestId) {
                sumTx += detections.get(i).getTargetXDegrees();
                sumTy += detections.get(i).getTargetYDegrees();
                count++;
            }
        }

        return new double[]{ sumTx / count, sumTy / count };
    }

    private static double angularDistance(
            LLResultTypes.DetectorResult a, LLResultTypes.DetectorResult b) {
        double dTx = a.getTargetXDegrees() - b.getTargetXDegrees();
        double dTy = a.getTargetYDegrees() - b.getTargetYDegrees();
        return Math.sqrt(dTx * dTx + dTy * dTy);
    }

    public static double[] cameraAnglesToFieldCoords(
            double txDeg, double tyDeg, SparkFunOTOS.Pose2D robotPose) {

        double t = Math.PI-robotPose.h;
        double a_r = -Math.toRadians(txDeg)+t;

        double c_x = robotPose.x - (9 * Math.cos(t));
        double c_y = robotPose.y + (9 * Math.sin(robotPose.h));

        return new double[]{ 8, c_y + c_x * Math.tan(a_r) };
    }

    public static double[] findLargestClumpFieldCoords(
            List<LLResultTypes.DetectorResult> detections,
            SparkFunOTOS.Pose2D robotPose,
            double threshold) {

        if (detections == null || detections.isEmpty()) return null;

        double[] centroidAngles = findLargestClumpCenter(detections, threshold);
        if (centroidAngles == null) return null;

        return cameraAnglesToFieldCoords(centroidAngles[0], centroidAngles[1], robotPose);
    }

}
