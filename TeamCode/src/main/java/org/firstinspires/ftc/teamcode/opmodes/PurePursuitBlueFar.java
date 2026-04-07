package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.opmodes.commands.CheckFollowerStatus;
import org.firstinspires.ftc.teamcode.opmodes.commands.FindAndTargetArtifactClump;
import org.firstinspires.ftc.teamcode.opmodes.commands.FraudInstantCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.FraudWaitCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.PurePursuitFollowPath;
import org.firstinspires.ftc.teamcode.opmodes.commands.SpinUpShooter;
import org.firstinspires.ftc.teamcode.opmodes.commands.SquiDCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.SquiDCommandTimeout;
import org.firstinspires.ftc.teamcode.systems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.PinpointConstants;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.systems.pureP.PurePursuitPath;
import org.firstinspires.ftc.teamcode.systems.pureP.PurePursuitSquidFollower;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDDrive;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDFollower;


@Config
@Autonomous
public class PurePursuitBlueFar extends OpMode {

    SquIDFollower follower; Shooter shooter; Intake intake; SquIDDrive drive; Turret turret;

    private GoBildaPinpointDriver pinpoint;

    public static double startPoseX = 41.5, startPoseY = 7.5, startHeadingRadians= Math.PI/2;

    public boolean pinpointReady = false;

    public static double sidespikeControlRadius = 9;

    CommandScheduler scheduler;

    CameraSubsystem camera;

    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(PinpointConstants.xDirection, PinpointConstants.yDirection);

        pinpoint.setOffsets(PinpointConstants.xOffset, PinpointConstants.yOffset, DistanceUnit.MM);

        drive = new SquIDDrive(hardwareMap, hardwareMap.voltageSensor.iterator().next());
        follower = new SquIDFollower(drive, pinpoint);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        turret = new Turret(hardwareMap, false);
        turret.setMode(Turret.Mode.odo);
        turret.setPose(new Pair<>(startPoseX, startPoseY), 90);
        turret.update();

        camera = new CameraSubsystem(hardwareMap);

        scheduler = CommandScheduler.getInstance();
        scheduler.reset();
        scheduler = CommandScheduler.getInstance();

        scheduler.schedule(
                new SequentialCommandGroup(
                        new FraudWaitCommand(900),
                        new IntakeCommand(intake, Intake.transferPosition, 1, true),
                        new WaitCommand(600),
                        new IntakeCommand(intake, Intake.lockedPosition, 1, false),
                        new SquiDCommand(follower, 15, 4,
                                new SparkFunOTOS.Pose2D(PathConstants.BlueSideSpikePath.x1, PathConstants.BlueSideSpikePath.y1, Math.toRadians(90))),
                        new SquiDCommand(follower, 8, 4,
                                new SparkFunOTOS.Pose2D(PathConstants.BlueSideSpikePath.x2, PathConstants.BlueSideSpikePath.y2, Math.toRadians(90))),
                        new SquiDCommandTimeout(follower, 8, 4,
                                new SparkFunOTOS.Pose2D(PathConstants.BlueReturnToFarPath.x1, PathConstants.BlueReturnToFarPath.y1, Math.toRadians(180)), 0.3),
                        new WaitCommand(400),
                        new IntakeCommand(intake, Intake.transferPosition, 1, true),
                        new WaitCommand(600),
                        new IntakeCommand(intake, Intake.lockedPosition, 1, false),
                        new SquiDCommand(follower, 8, 4,
                                new SparkFunOTOS.Pose2D(PathConstants.BlueCornerPath.x1, PathConstants.BlueCornerPath.y1, Math.toRadians(180))),
                        new SquiDCommandTimeout(follower, 8, 4,
                                new SparkFunOTOS.Pose2D(PathConstants.BlueReturnToFarPath.x1, PathConstants.BlueReturnToFarPath.y1, Math.toRadians(170)), 0.7),
                        new WaitCommand(400),
                        new IntakeCommand(intake, Intake.transferPosition, 1, true),
                        new ParallelCommandGroup(
                                new WaitCommand(600),
                                new FindAndTargetArtifactClump(camera, 600, follower, false, telemetry)
                        ),
                        new IntakeCommand(intake, Intake.lockedPosition, 1, false),
                        new CheckFollowerStatus(follower),
                        new SquiDCommandTimeout(follower, 8, 4,
                                new SparkFunOTOS.Pose2D(PathConstants.BlueReturnToFarPath.x1, PathConstants.BlueReturnToFarPath.y1, Math.toRadians(170)), 0.7),
                        new WaitCommand(400),
                        new IntakeCommand(intake, Intake.transferPosition, 1, true),
                        new ParallelCommandGroup(
                                new WaitCommand(600),
                                new FindAndTargetArtifactClump(camera, 600, follower, false, telemetry)
                        ),
                        new IntakeCommand(intake, Intake.lockedPosition, 1, false),
                        new CheckFollowerStatus(follower),
                        new SquiDCommandTimeout(follower, 8, 4,
                                new SparkFunOTOS.Pose2D(PathConstants.BlueReturnToFarPath.x1, PathConstants.BlueReturnToFarPath.y1, Math.toRadians(170)), 0.7),
                        new WaitCommand(400),
                        new IntakeCommand(intake, Intake.transferPosition, 1, true),
                        new ParallelCommandGroup(
                                new WaitCommand(600),
                                new FindAndTargetArtifactClump(camera, 600, follower, false, telemetry)
                        ),
                        new IntakeCommand(intake, Intake.lockedPosition, 1, false),
                        new CheckFollowerStatus(follower),
                        new SquiDCommandTimeout(follower, 8, 4,
                                new SparkFunOTOS.Pose2D(PathConstants.BlueReturnToFarPath.x1, PathConstants.BlueReturnToFarPath.y1, Math.toRadians(170)), 0.7),
                        new WaitCommand(400),
                        new IntakeCommand(intake, Intake.transferPosition, 1, true),
                        new ParallelCommandGroup(
                                new WaitCommand(600),
                                new FindAndTargetArtifactClump(camera, 600, follower, false, telemetry)
                        ),
                        new IntakeCommand(intake, Intake.lockedPosition, 1, false),
                        new CheckFollowerStatus(follower),
                        new SquiDCommandTimeout(follower, 8, 4,
                                new SparkFunOTOS.Pose2D(PathConstants.BlueReturnToFarPath.x1, PathConstants.BlueReturnToFarPath.y1, Math.toRadians(170)), 0.7),
                        new WaitCommand(400),
                        new IntakeCommand(intake, Intake.transferPosition, 1, true),
                        new ParallelCommandGroup(
                                new WaitCommand(600),
                                new FindAndTargetArtifactClump(camera, 600, follower, false, telemetry)
                        ),
                        new IntakeCommand(intake, Intake.lockedPosition, 1, false),
                        new CheckFollowerStatus(follower),
                        new SquiDCommandTimeout(follower, 8, 4,
                                new SparkFunOTOS.Pose2D(PathConstants.BlueReturnToFarPath.x1, PathConstants.BlueReturnToFarPath.y1, Math.toRadians(170)), 0.7),
                        new WaitCommand(400),
                        new IntakeCommand(intake, Intake.transferPosition, 1, true)
                )
        );

    }

    @Override
    public void init_loop() {
        if (!pinpointReady) {
            pinpoint.resetPosAndIMU();

            pinpoint.update();
        }
        if (pinpoint.getLoopTime() > 0) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, startPoseX, startPoseY, AngleUnit.RADIANS, startHeadingRadians));
            follower.setCurrentPose(startPoseX, startPoseY, startHeadingRadians);
            follower.setTargetPose(new SparkFunOTOS.Pose2D(startPoseX, startPoseY, startHeadingRadians));

            if (pinpointReady) {
                pinpoint.update();
            }
            pinpointReady = true;
        }
        telemetry.addData("headingRad", pinpoint.getHeading(AngleUnit.RADIANS));
        telemetry.update();
    }

    private double meters;

    @Override
    public void loop() {
        scheduler.run();

        follower.read();

        turret.setPose(new Pair<>(follower.getCurrentPose().x, follower.getCurrentPose().y), Math.toDegrees(follower.getCurrentPose().h));

        meters = turret.distanceToGoal(follower.getCurrentPose().x, follower.getCurrentPose().y) * 0.0254;

        shooter.updateFancyKinematics(meters);

        turret.update();
        follower.update();
        shooter.runShooterSus();


    }
}
