package org.firstinspires.ftc.teamcode.tests;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp
@Config
public class ArducamMadness extends OpMode {

    AprilTagProcessor processor;
    VisionPortal portal;

    public static AprilTagProcessor.PoseSolver solver = AprilTagProcessor.PoseSolver.APRILTAG_BUILTIN;

    @Override
    public void init() {

        processor = getProcessor();
        processor.setPoseSolver(solver);

        portal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "ardu"))
                .addProcessor(processor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .setAutoStartStreamOnBuild(true)
                .build();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (!processor.getDetections().isEmpty()) {
            for (AprilTagDetection detection : processor.getDetections()) {
                telemetry.addData("id: ", detection.id);
                telemetry.addData("pose x: ", detection.ftcPose.x);
                telemetry.addData("pose y: ", detection.ftcPose.y);
                telemetry.addData("pose z: ", detection.ftcPose.yaw);
                telemetry.update();
            }
        }

    }

    public AprilTagProcessor getProcessor() {
        return new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setLensIntrinsics(548.37, 548.50, 342.99, 262.04)
                .build();
    }
}
