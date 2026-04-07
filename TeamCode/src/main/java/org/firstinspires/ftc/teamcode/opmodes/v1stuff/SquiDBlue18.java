package org.firstinspires.ftc.teamcode.opmodes.v1stuff;

import static org.firstinspires.ftc.teamcode.tests.ShotAlgTest.c;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import java.util.List;

@Autonomous
@Config
@Disabled
public class SquiDBlue18 extends OpMode {

    SquIDFollower follower; SquIDDrive drive; boolean turretFlag = false;

    public static Pose2D endpose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS,0);

    Intake intake;

    Shooter shooter;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Gurt;

    CommandScheduler scheduler;

    Turret turret;

    List<LynxModule> allHubs;

    private GoBildaPinpointDriver pinpoint;
    public static GoBildaPinpointDriver.EncoderDirection yDirection, xDirection;

    public static double controlPointTol = 30, controlPointHeadingTol = 45;

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
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                        }),
                        new IntakeCommand(intake, Intake.transferPosition, 1),
                        new SquiDCommand(follower, 30, 50, new SparkFunOTOS.Pose2D(60, 110, Math.PI)),
                        new SquiDCommandTimeout(follower, 10, 2, new SparkFunOTOS.Pose2D(50, 84, Math.PI), 0.25),
                        new FraudInstantCommand(()->turretFlag = true),
                        new WaitCommand(300),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            intake.setTransfer(true);
                        }),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(true);
                            turret.update();
                        }),
                        new WaitCommand(600),
                        new ParallelCommandGroup(
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new FraudInstantCommand(()->turretFlag = false),
                        new ParallelCommandGroup(
                                new SquiDCommand(follower, 10, 2, new SparkFunOTOS.Pose2D(18, 84, Math.PI)),
                                new IntakeCommand(intake, Intake.lockedPosition, 1)
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new ParallelCommandGroup(
                                new SquiDCommandTimeout(follower, 10,2, new SparkFunOTOS.Pose2D(50, 80,Math.PI),0.25),
                                new IntakeCommand(intake, Intake.transferPosition, 1)
                        ),
                        new FraudInstantCommand(()->turretFlag = true),
                        new WaitCommand(400),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            intake.setTransfer(true);
                        }),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new FraudInstantCommand(()->{
                                            intake.setTransfer(false);
                                        }),
                                        new FraudInstantCommand(()->turretFlag = false)
                                ),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new SquiDCommand(follower, controlPointTol,controlPointHeadingTol, new SparkFunOTOS.Pose2D(54, 62, Math.PI)),
                                                new SquiDCommand(follower, 10,2, new SparkFunOTOS.Pose2D(9, 62, Math.PI))
                                        ),
                                        new IntakeCommand(intake, Intake.lockedPosition, 1)
                                )
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new SquiDCommand(follower, 25, 10, new SparkFunOTOS.Pose2D(50, 60, Math.PI)),
                        new FraudInstantCommand(()->turret.setOffset(-2)),
                        new SquiDCommandTimeout(follower, 10,2, new SparkFunOTOS.Pose2D(50, 80, Math.PI), 0.25),
                        new FraudInstantCommand(()->turretFlag = true),
                        new WaitCommand(400),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            intake.setTransfer(true);
                        }),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new FraudInstantCommand(()->{
                                            intake.setTransfer(false);
                                        }),
                                        new FraudInstantCommand(()->turretFlag = false)
                                ),
                                new SquiDCommand(follower, 8, 8, new SparkFunOTOS.Pose2D(13, 65, Math.toRadians(155)))
                        ),
                        new FraudInstantCommand(()->turret.setOffset(-2)),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new ParallelCommandGroup(
                                new WaitCommand(1500),
                                new SequentialCommandGroup(
                                        new WaitCommand(300),
                                        new SquiDCommand(follower, 8, 8, new SparkFunOTOS.Pose2D(8, 54, Math.toRadians(145))),
                                        new WaitCommand(800),
                                        new SquiDCommandTimeout(follower, 3, 5, new SparkFunOTOS.Pose2D(21, 55, Math.toRadians(180)), 0.4)
                                )
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new IntakeCommand(intake, Intake.lockedPosition, 0)
                                ),
                                new SquiDCommandTimeout(follower, 10,2, new SparkFunOTOS.Pose2D(50, 80, Math.PI), 0.25)
                        ),
                        new IntakeCommand(intake, Intake.lockedPosition, 1),
                        new FraudInstantCommand(()->turretFlag = true),
                        new WaitCommand(400),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            intake.setTransfer(true);
                        }),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new FraudInstantCommand(()->{
                                            intake.setTransfer(false);
                                        }),
                                        new FraudInstantCommand(()->turretFlag = false)
                                ),
                                new SquiDCommand(follower, 8, 8, new SparkFunOTOS.Pose2D(13, 65, Math.toRadians(155)))
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new ParallelCommandGroup(
                                new WaitCommand(1500),
                                new SequentialCommandGroup(
                                        new WaitCommand(800),
                                        new SquiDCommand(follower, 8, 8, new SparkFunOTOS.Pose2D(8, 54, Math.toRadians(145))),
                                        new WaitCommand(500),
                                        new SquiDCommandTimeout(follower, 3, 5, new SparkFunOTOS.Pose2D(15, 55, Math.toRadians(180)), 0.4)
                                )
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new IntakeCommand(intake, Intake.lockedPosition, 0)
                                ),
                                new SquiDCommandTimeout(follower, 10,2, new SparkFunOTOS.Pose2D(50, 80, Math.PI), 0.25)
                        ),
                        new IntakeCommand(intake, Intake.lockedPosition, 1),
                        new FraudInstantCommand(()->turretFlag = true),
                        new WaitCommand(400),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                }),
                                new FraudInstantCommand(()->turretFlag = false),
                                new IntakeCommand(intake, Intake.lockedPosition, 1)

                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new SquiDCommand(follower, 20, 15, new SparkFunOTOS.Pose2D(62, 38, Math.PI)),
                                        new SquiDCommand(follower, 10,2, new SparkFunOTOS.Pose2D(12, 38, Math.PI))
                                ),
                                new IntakeCommand(intake, Intake.lockedPosition, 1)
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(700),
                                        new IntakeCommand(intake, Intake.lockedPosition, 0)
                                ),
                                new SquiDCommandTimeout(follower, 10,2, new SparkFunOTOS.Pose2D(50, 80,Math.PI), 0.25)
                        ),
                        new IntakeCommand(intake, Intake.lockedPosition, 1),
                        new FraudInstantCommand(()->turretFlag = true),
                        new WaitCommand(400),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.lockedPosition, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new FraudInstantCommand(()->turretFlag = false),
                        new SquiDCommand(follower, 10,2, new SparkFunOTOS.Pose2D(32, 72, Math.PI)),
                        new StopShooter(shooter),
                        new IntakeCommand(intake, Intake.lockedPosition, 0)
                )
        );

        turret.setPose(new Pair<>(33.0, 135.0), 180);
        intake.setFlap(Intake.transferPosition);

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
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 35, 135, AngleUnit.RADIANS, Math.PI));
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

        shooter.setTargetRPM(shooter.getRPMForShot(meters) + c);
        shooter.setHoodAngle(shooter.getHoodAngle(meters));

        shooter.runShooterSus();

        PurpleAutoLimelight.endpose = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
        looptime = looptimer.milliseconds();


        telemetry.update();
    }

    @Override
    public void stop() {
        Pose pose = follower.getPose();
        PurpleAutoLimelight.endpose = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
    }

}
