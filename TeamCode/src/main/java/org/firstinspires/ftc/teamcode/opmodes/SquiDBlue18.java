package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.tests.ShotAlgTest.c;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
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
import org.firstinspires.ftc.teamcode.opmodes.commands.FraudWaitCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.PedroFollowCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.SquiDCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.StopShooter;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDFollower;

import java.util.List;

@Autonomous
@Config
public class SquiDBlue18 extends OpMode {

    SquIDFollower follower;

    public static Pose2D endpose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS,0);

    Intake intake;

    Shooter shooter;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Gurt;

    CommandScheduler scheduler;

    Turret turret;

    List<LynxModule> allHubs;

    private GoBildaPinpointDriver pinpoint;
    public static GoBildaPinpointDriver.EncoderDirection yDirection, xDirection;

    public static double controlPointTol = 8;

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

        pinpoint.resetPosAndIMU();

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 32, 135.6, AngleUnit.DEGREES, -90));

        scheduler = CommandScheduler.getInstance(); scheduler.reset(); scheduler = CommandScheduler.getInstance();

        follower.setTargetPose(follower.getCurrentPose());

        scheduler.schedule(
                new SequentialCommandGroup(
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                            turret.setOffset(3);
                        }),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new SquiDCommand(follower, 2, 2, new SparkFunOTOS.Pose2D(48, 84, Math.PI)),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new FraudWaitCommand(500),
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
                        new ParallelCommandGroup(
                                new SquiDCommand(follower, 2, 2, new SparkFunOTOS.Pose2D(18, 84, Math.PI)),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new ParallelCommandGroup(
                                new SquiDCommand(follower, 2,2, new SparkFunOTOS.Pose2D(54, 80, Math.PI)),
                                new IntakeCommand(intake, Intake.flapUp, 1)
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new WaitCommand(500),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new SquiDCommand(follower, controlPointTol,2, new SparkFunOTOS.Pose2D(54, 60, Math.PI)),
                                        new SquiDCommand(follower, 2,2, new SparkFunOTOS.Pose2D(18, 60, Math.PI))
                                ),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new SquiDCommand(follower, 2,2, new SparkFunOTOS.Pose2D(54, 80, Math.PI)),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new WaitCommand(500),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new PedroFollowCommand(follower, Path5),
//                                        new IntakeCommand(intake, Intake.flapDown, 1),
//                                        new PedroFollowCommand(follower, paths.PathHalf)
//                                ),
//                                new IntakeCommand(intake, Intake.flapDown, 1)
//                        ),
//                        new FraudInstantCommand(
//                                ()->turret.update()
//                        ),
//                        new WaitCommand(700),
////                        new FraudInstantCommand(()->{
////                            shooter.setTargetRPM(4300);
////                        }),
//                        new IntakeCommand(intake, Intake.flapDown, 0),
//                        new PedroFollowCommand(follower, Path6),
//                        new IntakeCommand(intake, Intake.flapDown, 1),
//                        new FraudInstantCommand(
//                                ()->turret.update()
//                        ),
//                        new WaitCommand(500),
//                        new FraudInstantCommand(()->{
//                            intake.setFlap(Intake.flapUp);
//                            intake.setTransfer(true);
//                        }),
//                        new WaitCommand(700),
//                        new ParallelCommandGroup(
//                                new IntakeCommand(intake, Intake.flapDown, 1),
//                                new FraudInstantCommand(()->{
//                                    intake.setTransfer(false);
//                                })
//                        ),
//                        new ParallelCommandGroup(
//                                new SequentialCommandGroup(
//                                        new PedroFollowCommand(follower, Path5),
//                                        new PedroFollowCommand(follower, paths.PathHalf)
//                                ),
//                                new IntakeCommand(intake, Intake.flapDown, 1)
//                        ),
//                        new FraudInstantCommand(
//                                ()->turret.update()
//                        ),
//                        new WaitCommand(700),
////                        new FraudInstantCommand(()->{
////                            shooter.setTargetRPM(4300);
////                        }),
//                        new IntakeCommand(intake, Intake.flapDown, 0),
//                        new PedroFollowCommand(follower, Path6),
//                        new IntakeCommand(intake, Intake.flapDown, 1),
//                        new FraudInstantCommand(
//                                ()->turret.update()
//                        ),
//                        new WaitCommand(300),
//                        new FraudInstantCommand(()->{
//                            intake.setFlap(Intake.flapUp);
//                            intake.setTransfer(true);
//                        }),
//                        new WaitCommand(700),
//                        new ParallelCommandGroup(
//                                new IntakeCommand(intake, Intake.flapDown, 1),
//                                new FraudInstantCommand(()->{
//                                    intake.setTransfer(false);
//                                })
//                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new SquiDCommand(follower, controlPointTol,2, new SparkFunOTOS.Pose2D(54, 35, Math.PI)),
                                        new SquiDCommand(follower, 2,2, new SparkFunOTOS.Pose2D(16, 35, Math.PI))
                                ),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new IntakeCommand(intake, Intake.flapDown, 0),
                        new SquiDCommand(follower, 2,2, new SparkFunOTOS.Pose2D(54, 80, Math.PI)),
                        new IntakeCommand(intake, Intake.flapDown, 1),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new WaitCommand(500),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new SquiDCommand(follower, 2,2, new SparkFunOTOS.Pose2D(32, 72, Math.PI)),
                        new StopShooter(shooter),
                        new IntakeCommand(intake, Intake.flapDown, 0)
                )
        );

        turret.setPose(new Pair<>(32.0, 135.6), -90);
        intake.setFlap(Intake.flapUp);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    private Pose pose; private double meters;

    double looptime = 0; boolean turretUpdateFlag = true;
    ElapsedTime looptimer = new ElapsedTime();
    ElapsedTime functionTimer = new ElapsedTime();

    @Override
    public void loop() {
        looptimer.reset();

        telemetry.addData("loop time (ms)", looptime);
        telemetry.addData("loop time (hz)", (1000/looptime));

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        follower.read();

        telemetry.addData("Telemetry, timer handling bulkread, pose ascription 2 time (ms)", functionTimer.milliseconds());

        functionTimer.reset();
        scheduler.run();
        telemetry.addData("CommandScheduler function time (ms)", functionTimer.milliseconds());

        functionTimer.reset();

        pose = follower.getPose();
        turret.setPose(new Pair<>(pose.getX(), pose.getY()), Math.toDegrees(pose.getHeading()));
        telemetry.addData("Pose ascription (ms)", functionTimer.milliseconds());

        functionTimer.reset();
        turret.update();
        telemetry.addData("Turret function time (ms)", functionTimer.milliseconds());

        functionTimer.reset();
        follower.update();
        telemetry.addData("Squid function time (ms)", functionTimer.milliseconds());

        functionTimer.reset();
        meters = turret.distanceToGoal(pose.getX(), pose.getY()) * 0.0254;
        shooter.setTargetRPM(shooter.getRPMForShot(meters) + c);
        shooter.setHoodAngle(shooter.getHoodAngle(meters));
        telemetry.addData("Shooter Calculation time", functionTimer.milliseconds());

        shooter.runShooter();
        telemetry.addData("RunShooter function time (ms)", shooter.getRunMs());

        functionTimer.reset();
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
