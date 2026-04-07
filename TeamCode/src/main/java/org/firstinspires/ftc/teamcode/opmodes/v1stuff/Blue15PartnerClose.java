package org.firstinspires.ftc.teamcode.opmodes.v1stuff;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.opmodes.commands.FraudInstantCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.SquiDCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.SquiDCommandTimeout;
import org.firstinspires.ftc.teamcode.opmodes.commands.StopShooter;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDDrive;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDFollower;

@Autonomous
public class Blue15PartnerClose extends OpMode {

    SquIDFollower follower; SquIDDrive drive; boolean turretFlag = false;

    public static Pose2D endpose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS,0);

    Intake intake;

    Shooter shooter;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Gurt;

    CommandScheduler scheduler;

    Turret turret;

    private GoBildaPinpointDriver pinpoint;
    public static GoBildaPinpointDriver.EncoderDirection yDirection, xDirection;

    public static double controlPointTol = 30, controlPointHeadingTol = 45, rpmBost = 1220;

    @Override
    public void init() {

        turret = new Turret(hardwareMap, false);
        turret.setMode(Turret.Mode.odo);

        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(xDirection, yDirection);

        pinpoint.setOffsets(-92.15, -111.625, DistanceUnit.MM);

        shooter.setHoodAngle(50);

        scheduler = CommandScheduler.getInstance(); scheduler.reset(); scheduler = CommandScheduler.getInstance();

        drive = new SquIDDrive(hardwareMap, hardwareMap.voltageSensor.iterator().next());
        follower = new SquIDFollower(drive, pinpoint);

        follower.setCurrentPose(33, 135, Math.PI);

        scheduler.schedule(
                new SequentialCommandGroup(
                        new IntakeCommand(intake, Intake.transferPosition, 1),
                        new SquiDCommand(follower, 30, 50, new SparkFunOTOS.Pose2D(60, 110, Math.PI)),
                        new SquiDCommandTimeout(follower, 10, 2, new SparkFunOTOS.Pose2D(50, 80, Math.PI), 0.25),

                        new FraudInstantCommand(()->rpmBost = 1300),

                        new WaitCommand(850),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(600),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(false);
                            intake.setFlap(Intake.lockedPosition);
                        }),

                        new FraudInstantCommand(()->rpmBost = 1100),

                        new SquiDCommand(follower, 20, 6, new SparkFunOTOS.Pose2D(16, 84, Math.toRadians(179))),
                        new SquiDCommand(follower, 10, 8, new SparkFunOTOS.Pose2D(10, 75, 0.5*Math.PI)),
                        new SquiDCommand(follower, 10, 4, new SparkFunOTOS.Pose2D(7, 75, 0.5*Math.PI)),
                        new WaitCommand(1500),
                        new SquiDCommandTimeout(follower, 10, 20, new SparkFunOTOS.Pose2D(50, 80, Math.PI), 0.25),

                        new WaitCommand(400),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(700),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(false);
                            intake.setFlap(Intake.lockedPosition);
                        }),

                        new SquiDCommand(follower, 4, controlPointHeadingTol, new SparkFunOTOS.Pose2D(50, 60, Math.PI)),
                        new SquiDCommand(follower, 15,9, new SparkFunOTOS.Pose2D(23, 60, Math.PI)),
                        new SquiDCommand(follower, 20,20, new SparkFunOTOS.Pose2D(13, 69, Math.toRadians(180))),

                        new WaitCommand(900),
                        new SquiDCommandTimeout(follower, 10,2, new SparkFunOTOS.Pose2D(50, 80, Math.PI), 0.25),

                        new WaitCommand(400),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(700),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(false);
                            intake.setFlap(Intake.lockedPosition);
                        }),

                        new FraudInstantCommand(()->turret.setOffset(3)),

                        new SquiDCommand(follower, 10, 15, new SparkFunOTOS.Pose2D(16, 70, Math.PI)),
                        new WaitCommand(200),
                        new SquiDCommand(follower, 10,15, new SparkFunOTOS.Pose2D(50, 80, Math.PI)),

                        new SquiDCommand(follower, 20, 15, new SparkFunOTOS.Pose2D(50, 38, Math.PI)),
                        new SquiDCommand(follower, 10,2, new SparkFunOTOS.Pose2D(10, 38, Math.PI)),
                        new SquiDCommandTimeout(follower, 10,2, new SparkFunOTOS.Pose2D(50, 80, Math.PI), 0.25),

                        new WaitCommand(400),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(700),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(false);
                            intake.setFlap(Intake.lockedPosition);
                        }),

                        new SquiDCommand(follower, 10, 15, new SparkFunOTOS.Pose2D(16, 70, Math.PI)),
                        new WaitCommand(1800),
                        new SquiDCommand(follower, 10,2, new SparkFunOTOS.Pose2D(28, 56, Math.PI)),
                        new SquiDCommand(follower, 10,2, new SparkFunOTOS.Pose2D(6, 38, Math.PI)),
                        new WaitCommand(250),
                        new SquiDCommand(follower, 10,2, new SparkFunOTOS.Pose2D(50, 80, Math.PI)),

                        new WaitCommand(400),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(700),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(false);
                            intake.setFlap(Intake.lockedPosition);
                        }),

                        new SquiDCommand(follower, 10,2, new SparkFunOTOS.Pose2D(32, 72, Math.PI)),
                        new StopShooter(shooter),
                        new IntakeCommand(intake, Intake.lockedPosition, 0)
                )
        );

        turret.setPose(new Pair<>(33.0, 135.0), 180);
        intake.setFlap(Intake.transferPosition);

        turret.setOffset(3);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

    }

    private Pose pose; private double meters;

    double looptime = 0; boolean pinpointReady = false;
    ElapsedTime looptimer = new ElapsedTime();
    ElapsedTime functionTimer = new ElapsedTime();

    @Override
    public void init_loop() {
        if (!pinpointReady) {
            pinpoint.resetPosAndIMU();

            pinpoint.update();
        }
        if (pinpoint.getLoopTime() > 0) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 33, 135, AngleUnit.RADIANS, Math.PI));
            follower.setCurrentPose(33, 135, Math.PI);
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

    @Override
    public void loop() {

        looptimer.reset();

        telemetry.addData("loop time (ms)", looptime);
        telemetry.addData("loop time (hz)", (1000/looptime));

        follower.read();

        functionTimer.reset();
        scheduler.run();

        pose = follower.getPose();
        turret.setPose(new Pair<>(pose.getX(), pose.getY()), Math.toDegrees(pose.getHeading()));

        follower.update();

        turret.update();

        meters = turret.distanceToGoal(pose.getX(), pose.getY()) * 0.0254;

        shooter.updateFancyKinematics(meters);
        shooter.setTargetRPM(shooter.getKinematicRPMGoal()+rpmBost+100);
        shooter.setHoodAngle(52);

        shooter.runShooterSus();

        looptime = looptimer.milliseconds();


        telemetry.update();
    }

    @Override
    public void stop() {
        Pose pose = follower.getPose();
        PurpleAutoLimelight.endpose = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
    }

}
