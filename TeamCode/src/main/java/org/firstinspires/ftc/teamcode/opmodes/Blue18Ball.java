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
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.opmodes.commands.FraudInstantCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.FraudWaitCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.PedroFollowCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.StopShooter;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;

import java.util.List;

@Autonomous
@Config
public class Blue18Ball extends OpMode {

    Follower follower;

    public static Pose2D endpose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS,0);

    Intake intake;

    Shooter shooter;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Gurt;

    Paths paths;

    CommandScheduler scheduler;

    Turret turret;

    List<LynxModule> allHubs;

    public static double backHalfBrakeTime = 8, breakStart = 0.5;

    @Override
    public void init() {

        turret = new Turret(hardwareMap, false);
        turret.setMode(Turret.Mode.odo);

        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        follower.setPose(new Pose(32, 135.600, Math.toRadians(-90)));

        paths = new Paths(follower);
        Path1 = paths.Path1; Path2 = paths.Path2; Path3 = paths.Path3; Path4 = paths.Path4;
        Path5 = paths.Path5; Path6 = paths.Path6; Path7 = paths.Path7; Path8 = paths.Path8;
        Path9 = paths.Path9; Gurt = paths.Gurt;

        scheduler = CommandScheduler.getInstance(); scheduler.reset(); scheduler = CommandScheduler.getInstance();
        shooter.setHoodAngle(50);

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
                        new PedroFollowCommand(follower, Path1),
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
                        new WaitCommand(700),
                        new ParallelCommandGroup(
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Gurt),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path2),
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
                                new PedroFollowCommand(follower, Path3),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path4)
                        ),
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
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new PedroFollowCommand(follower, Path5),
                                        new IntakeCommand(intake, Intake.flapDown, 1),
                                        new PedroFollowCommand(follower, paths.PathHalf)
                                ),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new WaitCommand(300),
//                        new FraudInstantCommand(()->{
//                            shooter.setTargetRPM(4300);
//                        }),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(1200),
                                        new IntakeCommand(intake, Intake.flapDown, 0)
                                        ),
                                new PedroFollowCommand(follower, Path6)
                        ),
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
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new PedroFollowCommand(follower, Path5),
                                        new PedroFollowCommand(follower, paths.PathHalf)
                                ),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new WaitCommand(300),
//                        new FraudInstantCommand(()->{
//                            shooter.setTargetRPM(4300);
//                        }),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new WaitCommand(1200),
                                        new IntakeCommand(intake, Intake.flapDown, 0)
                                ),
                                new PedroFollowCommand(follower, Path6)
                        ),
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
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path7),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new IntakeCommand(intake, Intake.flapDown, 0),
                        new PedroFollowCommand(follower, Path8),
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
                        new PedroFollowCommand(follower, Path9),
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

    double e1 = 0, e2 = 0, e3 = 0, e4 = 0, e5 = 0, loops =0;

    @Override
    public void loop() {
        looptimer.reset();

        telemetry.addData("loop time (ms)", looptime);
        telemetry.addData("loop time (hz)", (1000/looptime));

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

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
        e1 += functionTimer.milliseconds();

        functionTimer.reset();
        follower.update();
        telemetry.addData("Pedro function time (ms)", functionTimer.milliseconds());
        e2 += functionTimer.milliseconds();

        functionTimer.reset();
        meters = turret.distanceToGoal(pose.getX(), pose.getY()) * 0.0254;
        shooter.setTargetRPM(shooter.getRPMForShot(meters) + 1530);
        shooter.setHoodAngle(shooter.getHoodAngle(meters));
        telemetry.addData("Shooter Calculation time", functionTimer.milliseconds());
        e3 += functionTimer.milliseconds();

        shooter.runShooter();
        telemetry.addData("RunShooter function time (ms)", shooter.getRunMs());
        e4 += functionTimer.milliseconds();

        functionTimer.reset();
        PurpleAutoLimelight.endpose = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
        looptime = looptimer.milliseconds();
        telemetry.update();
        loops += 1;

        telemetry.addData("Turret function average time (ms)", e1/loops);
        telemetry.addData("Pedro function average time (ms)", e2/loops);
        telemetry.addData("Shooter calc function average time (ms)", e3/loops);
        telemetry.addData("Shooter function average time (ms)", e4/loops);
    }

    @Override
    public void stop() {
        Pose pose = follower.getPose();
        PurpleAutoLimelight.endpose = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());

    }


    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8, Path9, PathHalf, Gurt;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(32, 135.6), new Pose(55, 85)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                    .setVelocityConstraint(0)
                    .build();

            Gurt = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(48.25, 85), new Pose(19, 83)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19, 82.969), new Pose(55, 85))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .setVelocityConstraint(0)
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.25, 85),
                                    new Pose(56.8, 53.719),
                                    new Pose(19, 50)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(170))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19, 59), new Pose(55, 80))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setVelocityConstraint(0)
                    .build();


            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57, 75), new Pose(17, 70))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setVelocityConstraint(10)
                    .setTranslationalConstraint(2)
                    .build();

            PathHalf = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(17, 70), new Pose(11, 56))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(new BezierLine(new Pose(11, 53), new Pose(17, 53)))
                    .setLinearHeadingInterpolation(Math.toRadians(145), Math.toRadians(180))
                    .setHeadingConstraint(10)
                    .addPath(
                            new BezierLine(new Pose(11, 53), new Pose(55, 80))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setVelocityConstraint(0)
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57, 75),
                                    new Pose(58, 31),
                                    new Pose(15, 35)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(15, 35), new Pose(55, 80))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .setVelocityConstraint(0)
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57, 74.25), new Pose(36, 73))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();
        }
    }

    private double analogVoltageToDegrees(double voltage) {
        return voltage * (360/3.3);
    }
    public double getRPMForShot(double meters) {
//        return (211.43 * meters) + 1177;
        return (227.87*meters) + 1382.7;
    }

    public double getHoodAngle(double meters) {
//        return (-8.8 * meters) + 76.16;
        return (-4.8701*meters) + 59.754;
    }
}
