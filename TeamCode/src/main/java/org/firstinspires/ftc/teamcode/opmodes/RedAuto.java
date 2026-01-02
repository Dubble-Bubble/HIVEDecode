package org.firstinspires.ftc.teamcode.opmodes;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.opmodes.commands.FraudInstantCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.ManualShooterInputCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.PedroFollowCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.StopShooter;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;

@Autonomous
public class RedAuto extends OpMode {

    Follower follower;

    public static Pose2D endpose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS,0);

    Intake intake;

    Shooter shooter;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

    Paths paths;

    CommandScheduler scheduler;

    private DcMotor turret;
    private AnalogInput encoder;
    public double targetDeg = -90;

    public static double p = 0.025, i, d;

    @Override
    public void init() {

        turret = hardwareMap.dcMotor.get("turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoder = hardwareMap.analogInput.get("encoder");

        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        follower.setPose(new Pose(112, 135.600, Math.toRadians(-90)));

        paths = new Paths(follower);
        Path1 = paths.Path1; Path2 = paths.Path2; Path3 = paths.Path3; Path4 = paths.Path4;
        Path5 = paths.Path5; Path6 = paths.Path6; Path7 = paths.Path7; Path8 = paths.Path8;
        Path9 = paths.Path9;

        scheduler = CommandScheduler.getInstance(); scheduler.reset(); scheduler = CommandScheduler.getInstance();
        shooter.setHoodAngle(75);

        scheduler.schedule(
                new SequentialCommandGroup(
                        new FraudInstantCommand(()->{
                            shooter.setTargetRPM(2700);
                            targetDeg = -87;
                        }),
                        new WaitCommand(300),
                        new SequentialCommandGroup(
                                new IntakeCommand(intake, Intake.flapUp, 1),
                                new WaitCommand(1000),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new ParallelCommandGroup(
                                new FraudInstantCommand(()->{
                                    targetDeg = -56;
                                }),
                                new PedroFollowCommand(follower, Path1),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path2),
                                new FraudInstantCommand(()->{
                                    shooter.setTargetRPM(4400);
                                    shooter.setHoodAngle(45);
                                })
                        ),
                        new WaitCommand(300),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path3),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path4),
                                new FraudInstantCommand(()->{
                                    shooter.setTargetRPM(4400);
                                    shooter.setHoodAngle(45);
                                    targetDeg = -54;
                                })
                        ),
                        new WaitCommand(300),
                        new IntakeCommand(intake, Intake.flapUp, 71),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path5),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new WaitCommand(1300),
                        new FraudInstantCommand(()->{
                            shooter.setTargetRPM(4300);
                        }),
                        new PedroFollowCommand(follower, Path6),
                        new WaitCommand(300),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(1000),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path7),

                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new FraudInstantCommand(()->{
                            shooter.setTargetRPM(4400);
                            shooter.setHoodAngle(45);
                            targetDeg = -56.5;
                        }),
                        new PedroFollowCommand(follower, Path8),
                        new WaitCommand(300),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(1350),
                        new PedroFollowCommand(follower, Path9),
                        new StopShooter(shooter),
                        new IntakeCommand(intake, Intake.flapDown, 0)

//                        new ParallelCommandGroup(
//                                new ManualShooterInputCommand(shooter, 4800, 1.3, 55),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(400),
//                                        new IntakeCommand(intake, Intake.flapUp, 1)
//                                )
//                        ),
//                        new ParallelCommandGroup(
//                                new PedroFollowCommand(follower, Path5),
//                                new IntakeCommand(intake, Intake.flapDown, 1)
//                        ),
//                        new ParallelCommandGroup(
//                                new PedroFollowCommand(follower, Path6),
//                                new IntakeCommand(intake, Intake.flapDown, 0)
//                        ),
//                        new IntakeCommand(intake, Intake.flapUp, 1),
//                        new ParallelCommandGroup(
//                                new ManualShooterInputCommand(shooter, 6000, 1.3, 50),
//                                new SequentialCommandGroup(
//                                        new WaitCommand(400),
//                                        new IntakeCommand(intake, Intake.flapUp, 1)
//                                )
//                        )
                )
        );

        intake.setFlap(Intake.flapDown);

    }

    @Override
    public void loop() {
        scheduler.run();
        shooter.runShooter();
        follower.update();

        double turretDeg = analogVoltageToDegrees(encoder.getVoltage());

        double error = AngleUnit.normalizeDegrees(targetDeg-turretDeg);

        turret.setPower(p * error);
    }

    @Override
    public void stop() {
        Pose pose = follower.getPose();
        endpose = new Pose2D(DistanceUnit.INCH, pose.getX(), pose.getY(), AngleUnit.RADIANS, pose.getHeading());
    }


    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8, Path9;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(112.000, 135.600),
                                    new Pose(60.031, 84.438),
                                    new Pose(123.609, 82.969)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(123.609, 82.969), new Pose(94.078, 84.094))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(94.078, 84.094),
                                    new Pose(87.750, 53.719),
                                    new Pose(129.859, 56.484)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(129.859, 56.484), new Pose(88.172, 78.609))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();


            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.172, 80), new Pose(131, 61))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(55))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(131, 61), new Pose(88.172, 80))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(55), Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.172, 78.609),
                                    new Pose(70, 35.516),
                                    new Pose(135, 37)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(135, 37), new Pose(84.656, 73.828))
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
}
