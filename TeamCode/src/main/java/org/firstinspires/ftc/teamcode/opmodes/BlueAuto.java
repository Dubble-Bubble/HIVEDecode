package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.tests.ShotAlgTest.c;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

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

@Autonomous
public class BlueAuto extends OpMode {

    Follower follower;

    public static Pose2D endpose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS,0);

    Intake intake;

    Shooter shooter;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

    Paths paths;

    CommandScheduler scheduler;

    private DcMotor turret;
    private AnalogInput encoder;
    Limelight3A ll3a;
    public double targetDeg = 90;

    public static double p = 0.025, i, d;

    double limelightMountAngleDegrees = 10, limelightLensHeightInches = 13.4, goalHeightInches = 29.5, lastTurretTarget = 0;

    @Override
    public void init() {

        turret = hardwareMap.dcMotor.get("turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoder = hardwareMap.analogInput.get("encoder");

        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        follower.setPose(new Pose(32.5, 135.600, Math.toRadians(-90)));

        paths = new Paths(follower);
        Path1 = paths.Path1; Path2 = paths.Path2; Path3 = paths.Path3; Path4 = paths.Path4;
        Path5 = paths.Path5; Path6 = paths.Path6; Path7 = paths.Path7; Path8 = paths.Path8;
        Path9 = paths.Path9;

        scheduler = CommandScheduler.getInstance(); scheduler.reset(); scheduler = CommandScheduler.getInstance();
        shooter.setHoodAngle(75);

        scheduler.schedule(
                new SequentialCommandGroup(
                        new FraudInstantCommand(()->{
                            shooter.setTargetRPM(3750);
                            targetDeg = 87;
                        }),
                        new SequentialCommandGroup(
                                new IntakeCommand(intake, Intake.flapUp, 1),
                                new WaitCommand(1000),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new ParallelCommandGroup(
                                new FraudInstantCommand(()->{
                                    targetDeg = 45;
                                }),
                                new PedroFollowCommand(follower, Path1)
                        ),
                        new WaitCommand(200),
                        new IntakeCommand(intake, Intake.flapDown, 1),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path2),
                                new FraudInstantCommand(()->{
                                    shooter.setTargetRPM(4200);
                                    shooter.setHoodAngle(45);
                                })
                        ),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(900),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path3),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new WaitCommand(200),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path4),
                                new FraudInstantCommand(()->{
                                    shooter.setTargetRPM(4200);
                                    shooter.setHoodAngle(45);
                                    targetDeg = 46;
                                }),
                                new SequentialCommandGroup(
                                        new WaitCommand(100),
                                        new IntakeCommand(intake, Intake.flapDown, 1)
                                )
                        ),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(900),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path5),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new WaitCommand(1300),
                        new FraudInstantCommand(()->{
                            targetDeg = 47.5;
                            shooter.setTargetRPM(4400);
                        }),
                        new PedroFollowCommand(follower, Path6),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(800),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path7),

                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new FraudInstantCommand(()->{
                            shooter.setTargetRPM(4400);
                            shooter.setHoodAngle(45);
                            targetDeg = 95.5;
                        }),
                        new PedroFollowCommand(follower, Path8),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(800),
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
            double turretDeg = analogVoltageToDegrees(encoder.getVoltage());

            double error = AngleUnit.normalizeDegrees(targetDeg-turretDeg);

            turret.setPower(p * error);
        shooter.runShooter();
        follower.update();


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
        public PathChain Path8, Path9,InterPath;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(32.500, 135.600),
                                    new Pose(78.469, 89.438),
                                    new Pose(16, 86)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(16, 86), new Pose(48.234, 85.078))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(48.234, 85.078),
                                    new Pose(56.813, 51.750),
                                    new Pose(18.703, 60.328)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.703, 60.328), new Pose(57.094, 75.234))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.094, 75.234),
                                    new Pose(36.141, 62.438),
                                    new Pose(11.406, 62)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(150))
                    .build();

            InterPath = follower.pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.406, 62), new Pose(10.406, 59))
                    )
                    .setConstantHeadingInterpolation(145)
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(11.406, 62), new Pose(57.797, 70.875))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(150), Math.toRadians(180))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(57.797, 70.875),
                                    new Pose(57.234, 31.500),
                                    new Pose(18.984, 35.297)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(18.984, 35.297), new Pose(57.094, 74.250))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(225))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.094, 74.250), new Pose(36.141, 73.688))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(225), Math.toRadians(180))
                    .build();
        }
    }

    private double analogVoltageToDegrees(double voltage) {
        return voltage * (360/3.3);
    }

    public double getRPMForShot(double meters) {
        return (211.43 * meters) + 1177;
    }

    public double getHoodAngle(double meters) {
        return (-8.8 * meters) + 76.16;
    }
}
