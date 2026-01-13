package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.opmodes.PurpleAutoLimelight;
import org.firstinspires.ftc.teamcode.opmodes.commands.FraudInstantCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.FraudWaitCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.PedroFollowCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.StopShooter;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;

@Autonomous
public class BlueFarAuto extends OpMode {

    Follower follower;

    public static Pose2D endpose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS,0);

    Intake intake;

    Shooter shooter;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Gurt;

    Paths paths;

    CommandScheduler scheduler;

    Turret turret;

    @Override
    public void init() {

        turret = new Turret(hardwareMap, false);
        turret.setMode(Turret.Mode.odo);

        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        follower.setPose(new Pose(56, 8, Math.toRadians(180)));

        paths = new Paths(follower);
        Path1 = paths.Path1; Path2 = paths.Path2; Path3 = paths.Path3;
        Path4 = paths.Path4; Path5 = paths.Path5; Path6 = paths.Path6;

        scheduler = CommandScheduler.getInstance(); scheduler.reset(); scheduler = CommandScheduler.getInstance();
        shooter.setHoodAngle(50);

        scheduler.schedule(
                new SequentialCommandGroup(
                        new FraudWaitCommand(1100),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new FraudInstantCommand(()->
                                intake.setTransfer(true)
                        ),
                        new WaitCommand(700),
                        new FraudInstantCommand(()->{
                                intake.setTransfer(false);
                                intake.setFlap(Intake.flapDown);
                                turret.setOffset(-1.5);
                            }
                        ),
                        new PedroFollowCommand(follower, Path1),
                        new PedroFollowCommand(follower, Path2),
                        new WaitCommand(200),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new FraudInstantCommand(()->
                                intake.setTransfer(true)
                        ),
                        new WaitCommand(700),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(false);
                            intake.setFlap(Intake.flapDown);
                            turret.setOffset(-3.25);
                        }),
                        new PedroFollowCommand(follower, Path3),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path4),
                                new SequentialCommandGroup(
                                        new FraudWaitCommand(400),
                                        new IntakeCommand(intake, Intake.flapDown, 0)
                                )
                        ),
                        new WaitCommand(200),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new FraudInstantCommand(()->
                                intake.setTransfer(true)
                        ),
                        new WaitCommand(700),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(false);
                            intake.setFlap(Intake.flapDown);
                        }),
                        new FraudInstantCommand(()-> turret.setOffset(-1)),
                        new FraudWaitCommand(1500),
                        new PedroFollowCommand(follower, Path1),
                        new IntakeCommand(intake, Intake.flapDown, 0),
                        new PedroFollowCommand(follower, Path2),
                        new WaitCommand(200),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new FraudInstantCommand(()->
                                intake.setTransfer(true)
                        ),
                        new WaitCommand(700),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(false);
                            intake.setFlap(Intake.flapDown);
                        }),
                        new PedroFollowCommand(follower, Path1),
                        new IntakeCommand(intake, Intake.flapDown, 0),
                        new WaitCommand(200),
                        new PedroFollowCommand(follower, Path2),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new FraudInstantCommand(()->
                                intake.setTransfer(true)
                        ),
                        new WaitCommand(700),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(false);
                            intake.setFlap(Intake.flapDown);
                        }),
                        new IntakeCommand(intake, Intake.flapDown, 0),
                        new PedroFollowCommand(follower, Path1)
                )
        );

        turret.setPose(new Pair<>(56.0, 8.0), 180);
        turret.update();
        intake.setFlap(Intake.flapDown);
        turret.setOffset(-1);
    }

    private Pose pose; private double meters;

    @Override
    public void loop() {
        scheduler.run();
        follower.update();
        pose = follower.getPose();

        turret.setPose(new Pair<>(pose.getX(), pose.getY()), Math.toDegrees(pose.getHeading()));

        meters = turret.distanceToGoal(pose.getX(), pose.getY()) * 0.0254;
        shooter.setTargetRPM(shooter.getRPMForShot(meters)+1900);
        shooter.setHoodAngle(shooter.getHoodAngle(meters));

        shooter.runShooter();
        turret.update();

        PurpleAutoLimelight.endpose = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
    }

    @Override
    public void stop() {
        Pose pose = follower.getPose();
        PurpleAutoLimelight.endpose = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
        turret.setOffset(0);
    }


    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(7.5, 8.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setTranslationalConstraint(12)
                    .setVelocityConstraint(20)
                    .setTimeoutConstraint(0)
                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10, 8.5),

                                    new Pose(56.000, 8.5)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .setBrakingStart(3)
                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(56.000, 8.5),
                                    new Pose(56.000, 38),
                                    new Pose(18, 40)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(18, 35.959),

                                    new Pose(56.000, 8.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .setBrakingStart(3)
                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.500),

                                    new Pose(56.000, 8.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .setBrakingStart(3)
                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.500),

                                    new Pose(10.408, 24.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))
                    .setBrakingStart(3)
                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.408, 24.000),

                                    new Pose(56.000, 8.500)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();
        }
    }

}
