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
import org.firstinspires.ftc.teamcode.opmodes.commands.FraudInstantCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.PedroFollowCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.StopShooter;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;

@Autonomous
public class Red18Ball extends OpMode {

    Follower follower;

    public static Pose2D endpose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS,0);
    Turret turret;

    Intake intake;

    Shooter shooter;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path0;

    Paths paths;

    CommandScheduler scheduler;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        follower.setPose(new Pose(112, 135.600, Math.toRadians(-90)));

        paths = new Paths(follower);
        Path1 = paths.Path1; Path2 = paths.Path2; Path3 = paths.Path3; Path4 = paths.Path4;
        Path5 = paths.Path5; Path6 = paths.Path6; Path7 = paths.Path7; Path8 = paths.Path8;
        Path9 = paths.Path9; Path0 = paths.Path0;

        scheduler = CommandScheduler.getInstance(); scheduler.reset(); scheduler = CommandScheduler.getInstance();
        shooter.setHoodAngle(50);

        scheduler.schedule(
                new SequentialCommandGroup(
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                        }),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new PedroFollowCommand(follower, Path0),
                        new WaitCommand(250),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(500),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                    turret.setOffset(-4);
                                })
                        ),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path1),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path2),
                                new IntakeCommand(intake, Intake.flapUp, 1)
                        ),
                        new WaitCommand(400),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(600),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                    turret.setOffset(0);
                                })
                        ),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path3),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path4)
                        ),
                        new WaitCommand(250),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(600),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new PedroFollowCommand(follower, Path5),
                                        new WaitCommand(100),
                                        new PedroFollowCommand(follower, paths.PathHalf)
                                ),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new WaitCommand(1000),
//                        new FraudInstantCommand(()->{
//                            shooter.setTargetRPM(4300);
//                        }),
                        new IntakeCommand(intake, Intake.flapDown, 0),
                        new PedroFollowCommand(follower, Path6),
                        new IntakeCommand(intake, Intake.flapDown, 1),
                        new WaitCommand(250),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(600),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new PedroFollowCommand(follower, Path5),
                                        new WaitCommand(100),
                                        new PedroFollowCommand(follower, paths.PathHalf)
                                ),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new WaitCommand(1000),
//                        new FraudInstantCommand(()->{
//                            shooter.setTargetRPM(4300);
//                        }),
                        new IntakeCommand(intake, Intake.flapDown, 0),
                        new PedroFollowCommand(follower, Path6),
                        new IntakeCommand(intake, Intake.flapDown, 1),
                        new WaitCommand(250),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(600),
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
                        new IntakeCommand(intake, Intake.flapDown, 0),
                        new PedroFollowCommand(follower, Path8),
                        new IntakeCommand(intake, Intake.flapDown, 1),
                        new WaitCommand(250),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(600),
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

        intake.setFlap(Intake.flapUp);

        turret = new Turret(hardwareMap, true);
        turret.setMode(Turret.Mode.odo);

    }

    private Pose pose;
    double meters;

    @Override
    public void loop() {
        scheduler.run();
        follower.update();
        pose = follower.getPose();

        turret.setPose(new Pair<>(pose.getX(), pose.getY()), Math.toDegrees(pose.getHeading()));

        meters = turret.distanceToGoal(pose.getX(), pose.getY()) * 0.0254;
        shooter.setTargetRPM(shooter.getRPMForShot(meters)+1350);
        shooter.setHoodAngle(shooter.getHoodAngle(meters));

        shooter.runShooter();
        turret.update();

        RedAutoLimelight.endpose = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
    }

    @Override
    public void stop() {
        Pose pose = follower.getPose();
        RedAutoLimelight.endpose = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
    }


    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8, Path9, PathHalf, Path0;

        public Paths(Follower follower) {

            Path0 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(112, 135.5), new Pose(94, 86))
                    ) .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .build();

            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(94, 86), new Pose(124, 82))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(124, 82.969), new Pose(94.078, 84.094))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .setVelocityConstraint(0)
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(94.078, 84.094),
                                    new Pose(87.750, 53.719),
                                    new Pose(124, 56.484)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(124, 56.484), new Pose(88.172, 80))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();


            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.172, 80), new Pose(125, 65))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .setVelocityConstraint(10)
                    .setTranslationalConstraint(2)
                    .build();

            PathHalf = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(125, 65), new Pose(133, 49))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(133, 49), new Pose(88.172, 80))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.172, 78.609),
                                    new Pose(70, 35.516),
                                    new Pose(133, 33)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(133, 33), new Pose(84.656, 73.828))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(84.656, 73.828), new Pose(95, 73.828))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
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
