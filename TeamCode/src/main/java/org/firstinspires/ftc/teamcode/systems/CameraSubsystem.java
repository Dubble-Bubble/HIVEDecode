package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Config
public class CameraSubsystem {

    private Limelight3A limelight;
    public static double limelightForward = 9, threshold = 8;

    public CameraSubsystem(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(2);
    }

    LLResult result;
    List<LLResultTypes.DetectorResult> detections;

    double[] templist;

    double headingTarget = 0;

    public SparkFunOTOS.Pose2D getClumpLocation(SparkFunOTOS.Pose2D robotPose, boolean red) {
        headingTarget = red ? 0 : 180;
        result = limelight.getLatestResult();
        detections = result.getDetectorResults();

        templist = findLargestClumpFieldCoords(detections, robotPose, threshold);

        if (templist == null) {
            return new SparkFunOTOS.Pose2D(9, 9, Math.toRadians(headingTarget));
        } else {
            if (templist[1] < 9) {
                return new SparkFunOTOS.Pose2D(9, 9, Math.toRadians(headingTarget));
            } else {
                return new SparkFunOTOS.Pose2D(8, templist[1], Math.toRadians(headingTarget));
            }
        }
    }

    private double[] findLargestClumpCenter(
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

    private double[] cameraAnglesToFieldCoords(
            double txDeg, double tyDeg, SparkFunOTOS.Pose2D robotPose) {

        double t = Math.PI-robotPose.h;
        double a_r = -Math.toRadians(txDeg)+t;

        double c_x = robotPose.x - (limelightForward * Math.cos(t));
        double c_y = robotPose.y + (limelightForward * Math.sin(robotPose.h));

        return new double[]{ 8, c_y + c_x * Math.tan(a_r) };
    }

    public double[] findLargestClumpFieldCoords(
            List<LLResultTypes.DetectorResult> detections,
            SparkFunOTOS.Pose2D robotPose,
            double threshold) {

        if (detections == null || detections.isEmpty()) return null;

        double[] centroidAngles = findLargestClumpCenter(detections, threshold);
        if (centroidAngles == null) return null;

        return cameraAnglesToFieldCoords(centroidAngles[0], centroidAngles[1], robotPose);
    }

}
