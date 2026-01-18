package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.tests.ShotAlgTest.c;

import android.util.Pair;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.opmodes.PurpleAutoLimelight;
import org.firstinspires.ftc.teamcode.opmodes.commands.FraudInstantCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.PedroFollowCommand;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;

@Autonomous
public class AmeyAutoBlue extends OpMode {

    Follower follower; Intake intake; Shooter shooter;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10, PathGurt, Path11, Path12;

    Paths paths;

    CommandScheduler scheduler;

    Turret turret;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, false);

        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        paths = new Paths(follower);
        Path1 = paths.Path1; Path2 = paths.Path2; Path3 = paths.Path3; Path4 = paths.Path4; Path5 = paths.Path5; Path6 = paths.Path6;
        Path7 = paths.Path7; Path8 = paths.Path8; Path9 = paths.Path9;

        follower.setPose(new Pose(144-112, 134.0, Math.toRadians(-90)));

        scheduler = CommandScheduler.getInstance(); scheduler.reset(); scheduler = CommandScheduler.getInstance();

        shooter.setHoodAngle(50);

        scheduler.schedule(
                new SequentialCommandGroup(
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                        }),
                        new FraudInstantCommand(()->turret.update()),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapUp, 1),
                                new PedroFollowCommand(follower, Path1)
                        ),
                        new FraudInstantCommand(()->turret.update()),
                        new FraudInstantCommand(()->turret.update()),
                        new WaitCommand(300),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(650),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new PedroFollowCommand(follower, Path2),
                        new FraudInstantCommand(()->turret.update()),
                        new WaitCommand(600),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                        }),
                        new PedroFollowCommand(follower, Path3),
                        new FraudInstantCommand(()->turret.update()),
                        new WaitCommand(300),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(760),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),

                        new PedroFollowCommand(follower, Path4),
                        new FraudInstantCommand(()->turret.update()),
                        new WaitCommand(600),
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                        }),
                        new PedroFollowCommand(follower, Path5),
                        new FraudInstantCommand(()->turret.update()),
                        new WaitCommand(200),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(760),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new PedroFollowCommand(follower, Path6),
                        new FraudInstantCommand(()->turret.update()),
                        new WaitCommand(600),
                        new PedroFollowCommand(follower, Path7),
                        new IntakeCommand(intake, Intake.flapDown, 1),
                        new PedroFollowCommand(follower, Path8),
                        new FraudInstantCommand(()->turret.update()),
                        new WaitCommand(300),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(760),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new FraudInstantCommand(()->turret.update()),
                        new PedroFollowCommand(follower, Path9)
                )
        );
        turret.setMode(Turret.Mode.odo);
    }

    private Pose pose;
    private double meters;

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
        public PathChain Path8;
        public PathChain Path9;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(32.000, 134.000),

                                    new Pose(49.020, 83.204)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(49.020, 83.204),
                                    new Pose(24.796, 90.551),
                                    new Pose(15.878, 69.939)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.878, 69.939),

                                    new Pose(48.878, 83.347)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(48.878, 83.347),
                                    new Pose(56.806, 56.398),
                                    new Pose(16.592, 57.194),
                                    new Pose(15.694, 65.163)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.694, 65.163),

                                    new Pose(48.816, 83.347)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.816, 83.347),

                                    new Pose(15.714, 69.633)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(15.714, 69.633),
                                    new Pose(85.133, 36.122),
                                    new Pose(19.082, 35.755)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path8 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(19.082, 35.755),

                                    new Pose(49.020, 82.939)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(233))

                    .build();

            Path9 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(49.020, 82.939),

                                    new Pose(16.224, 70.061)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(233), Math.toRadians(180))

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


