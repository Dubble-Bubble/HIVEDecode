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
import org.firstinspires.ftc.teamcode.opmodes.commands.FraudWaitCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.PedroFollowCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.StopShooter;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;

@Autonomous
public class BluePhantomAuto extends OpMode {

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
                            turret.setOffset(-1);
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
                        new FraudWaitCommand(300),
                        new FraudInstantCommand(()->{
                            intake.setTransferSlower(true);
                            turret.update();
                        }),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                                new FraudInstantCommand(()->{
                                    intake.setTransferSlower(false);
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
                        new WaitCommand(300),
                        new FraudInstantCommand(()->{
                            intake.setTransferSlower(true);
                        }),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                                new FraudInstantCommand(()->{
                                    intake.setTransferSlower(false);
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
                        new WaitCommand(300),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                            intake.setTransferSlower(true);
                        }),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                                new FraudInstantCommand(()->{
                                    intake.setTransferSlower(false);
                                })
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new PedroFollowCommand(follower, Path5),
                                        new WaitCommand(100),
                                        new IntakeCommand(intake, Intake.flapDown, 1),
                                        new PedroFollowCommand(follower, paths.PathHalf)
                                ),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new WaitCommand(900),
//                        new FraudInstantCommand(()->{
//                            shooter.setTargetRPM(4300);
//                        }),
                        new IntakeCommand(intake, Intake.flapDown, 0),
                        new PedroFollowCommand(follower, Path6),
                        new IntakeCommand(intake, Intake.flapDown, 1),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new WaitCommand(300),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                            intake.setTransferSlower(true);
                        }),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransferSlower(false);
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
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new WaitCommand(900),
//                        new FraudInstantCommand(()->{
//                            shooter.setTargetRPM(4300);
//                        }),
                        new IntakeCommand(intake, Intake.flapDown, 0),
                        new PedroFollowCommand(follower, Path6),
                        new IntakeCommand(intake, Intake.flapDown, 1),
                        new FraudInstantCommand(
                                ()->turret.update()
                        ),
                        new WaitCommand(300),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                            intake.setTransferSlower(true);
                        }),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransferSlower(false);
                                })
                        ),
                        new PedroFollowCommand(follower, Path9),
                        new StopShooter(shooter),
                        new IntakeCommand(intake, Intake.flapDown, 0)
                )
        );

        turret.setPose(new Pair<>(32.0, 135.6), -90);
        intake.setFlap(Intake.flapUp);
    }

    private Pose pose; private double meters;

    @Override
    public void loop() {
        scheduler.run();
        follower.update();
        pose = follower.getPose();

        turret.setPose(new Pair<>(pose.getX(), pose.getY()), Math.toDegrees(pose.getHeading()));

        meters = turret.distanceToGoal(pose.getX(), pose.getY()) * 0.0254;
        shooter.setTargetRPM(shooter.getRPMForShot(meters)+1250);
        shooter.setHoodAngle(shooter.getHoodAngle(meters));

        shooter.runShooter();

        PurpleAutoLimelight.endpose = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
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
                            new BezierLine(new Pose(32, 135.6), new Pose(48.25, 85)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
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
                            new BezierLine(new Pose(19, 82.969), new Pose(48.25, 85))
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
                                    new Pose(19, 59)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(19, 59), new Pose(57, 75))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                            new BezierLine(new Pose(17, 70), new Pose(11, 53))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(11, 53), new Pose(57, 75))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
                            new BezierLine(new Pose(15, 35), new Pose(57, 74.25))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
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
