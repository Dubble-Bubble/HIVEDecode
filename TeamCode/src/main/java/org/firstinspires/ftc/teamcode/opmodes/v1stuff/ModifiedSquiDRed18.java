package org.firstinspires.ftc.teamcode.opmodes.v1stuff;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
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

import java.util.List;

@Autonomous
@Config
public class ModifiedSquiDRed18 extends OpMode {

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

        turret = new Turret(hardwareMap, true);
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

        follower.setCurrentPose(144-33, 135, 0);

        scheduler.schedule(
                new SequentialCommandGroup(
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.transferPosition);
                            turret.setOffset(3);
                        }),
                        new IntakeCommand(intake, Intake.transferPosition, 1),
                        new SquiDCommand(follower, 30, 50, new SparkFunOTOS.Pose2D(144-60, 110, 0)),
                        new SquiDCommandTimeout(follower, 10, 2, new SparkFunOTOS.Pose2D(144-50, 80, 0), 0.25),
                        new FraudInstantCommand(()->turretFlag = true),
                        new WaitCommand(700),
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
                                new SequentialCommandGroup(
                                        new FraudInstantCommand(()->{
                                            intake.setTransfer(false);
                                        }),
                                        new FraudInstantCommand(()->turretFlag = false),
                                        new FraudInstantCommand(()-> turret.setOffset(2))
                                ),
                                new ParallelCommandGroup(
                                        new SequentialCommandGroup(
                                                new SquiDCommand(follower, 4, controlPointHeadingTol, new SparkFunOTOS.Pose2D(144-58, 62, 0)),
                                                new SquiDCommand(follower, 10,2, new SparkFunOTOS.Pose2D(144-8, 62, 0))
                                        ),
                                        new IntakeCommand(intake, Intake.lockedPosition, 1)
                                )
                        ),
                        new FraudInstantCommand(
                                ()-> {
                                    turret.update();
                                }
                        ),
                        new SquiDCommand(follower, 25, 10, new SparkFunOTOS.Pose2D(144-50, 60, 0)),
                        new FraudInstantCommand(()->turret.setOffset(3)),
                        new SquiDCommandTimeout(follower, 10,2, new SparkFunOTOS.Pose2D(144-50, 80, 0), 0.25),
                        new FraudInstantCommand(()->turretFlag = true),
                        new WaitCommand(500),
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
                                new SquiDCommand(follower, 8, 8, new SparkFunOTOS.Pose2D(144-13, 65, Math.toRadians(25)))
                        ),
                        new FraudInstantCommand(()->turret.setOffset(3)),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SquiDCommand(follower, 8, 8, new SparkFunOTOS.Pose2D(144-8, 54, Math.toRadians(35))),
                                        new WaitCommand(500),
                                        new ParallelRaceGroup(
                                                new SquiDCommandTimeout(follower, 0.2, 0.2, new SparkFunOTOS.Pose2D(144-8, 56, Math.toRadians(0)), 0.6),
                                                new WaitCommand(400)
                                        )
                                )
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new IntakeCommand(intake, Intake.lockedPosition, 0)
                                ),
                                new SequentialCommandGroup(
                                        new SquiDCommand(follower,10, controlPointHeadingTol, new SparkFunOTOS.Pose2D(144-45, 56, 0)),
                                        new SquiDCommandTimeout(follower, 10,2, new SparkFunOTOS.Pose2D(144-50, 80, 0), 0.25)
                                )
                        ),
                        new IntakeCommand(intake, Intake.lockedPosition, 1),
                        new FraudInstantCommand(()->turretFlag = true),
                        new WaitCommand(500),
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
                                )
                        ),
                        new FraudInstantCommand(()->turretFlag = false),
                        new ParallelCommandGroup(
                                new SquiDCommand(follower, 10, 2, new SparkFunOTOS.Pose2D(144-18, 84, 0)),
                                new IntakeCommand(intake, Intake.lockedPosition, 1)
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new ParallelCommandGroup(
                                new SquiDCommandTimeout(follower, 10,2, new SparkFunOTOS.Pose2D(144-50, 80, 0),0.25),
                                new IntakeCommand(intake, Intake.transferPosition, 1)
                        ),
                        new FraudInstantCommand(()->turretFlag = true),
                        new WaitCommand(500),
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
                                                new SquiDCommand(follower, 8, 8, new SparkFunOTOS.Pose2D(144-13, 67, Math.toRadians(25)))
                                        ),
                                        new IntakeCommand(intake, Intake.lockedPosition, 1)
                                )
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(500),
                                        new SquiDCommand(follower, 8, 8, new SparkFunOTOS.Pose2D(144-8, 54, Math.toRadians(35))),
                                        new WaitCommand(500),
                                        new ParallelRaceGroup(
                                                new SquiDCommandTimeout(follower, 0.2, 0.2, new SparkFunOTOS.Pose2D(144-8, 56, Math.toRadians(0)), 0.6),
                                                new WaitCommand(500)
                                        )
                                )
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(600),
                                        new IntakeCommand(intake, Intake.lockedPosition, 0)
                                ),
                                new SquiDCommandTimeout(follower, 10,2, new SparkFunOTOS.Pose2D(144-50, 80, 0), 0.25)
                        ),
                        new IntakeCommand(intake, Intake.lockedPosition, 1),
                        new FraudInstantCommand(()->turretFlag = true),
                        new WaitCommand(500),
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
                                        new SquiDCommand(follower, 20, 15, new SparkFunOTOS.Pose2D(144-62, 38, 0)),
                                        new SquiDCommand(follower, 10,2, new SparkFunOTOS.Pose2D(144-12, 38, 0))
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
                                new SquiDCommandTimeout(follower, 10,2, new SparkFunOTOS.Pose2D(144-50, 80, 0), 0.25)
                        ),
                        new IntakeCommand(intake, Intake.lockedPosition, 1),
                        new FraudInstantCommand(()->turretFlag = true),
                        new WaitCommand(500),
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
                        new SquiDCommand(follower, 10,2, new SparkFunOTOS.Pose2D(144-32, 72, 0)),
                        new StopShooter(shooter),
                        new IntakeCommand(intake, Intake.lockedPosition, 0)
                )
        );

        turret.setPose(new Pair<>(144-33.0, 135.0), 0);
        intake.setFlap(Intake.transferPosition);

        turret.setOffset(2);

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
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 144-33, 135, AngleUnit.RADIANS, 0));
            follower.setCurrentPose(144-33, 135, 0);
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
        shooter.setTargetRPM(shooter.getRPMForShot(meters) + ModifiedBlueSquid18.rpmBost);
        shooter.setHoodAngle(shooter.getHoodAngle(meters));

        shooter.runShooterSus();

        RedAutoLimelight.endpose = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
        looptime = looptimer.milliseconds();


        telemetry.update();
    }

    @Override
    public void stop() {
        Pose pose = follower.getPose();
        RedAutoLimelight.endpose = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
    }

}
