package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.tests.ShotAlgTest.c;

import com.arcrobotics.ftclib.command.CommandScheduler;
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
import org.firstinspires.ftc.teamcode.opmodes.RedAutoLimelight;
import org.firstinspires.ftc.teamcode.opmodes.commands.FraudInstantCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.PedroFollowCommand;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;

@Autonomous
public class AmeyAutoRed extends OpMode {

    Follower follower; Intake intake; Shooter shooter;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9, Path10;

    Paths paths;

    CommandScheduler scheduler;

    private DcMotor turret;
    private AnalogInput encoder;
    public double targetDeg = -90;

    public static double p = 0.025, i, d;

    Limelight3A ll3a;
    double limelightMountAngleDegrees = 10, limelightLensHeightInches = 13.4, goalHeightInches = 29.5, lastTurretTarget = 0;

    @Override
    public void init() {
        turret = hardwareMap.dcMotor.get("turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        encoder = hardwareMap.analogInput.get("encoder");

        ll3a = hardwareMap.get(Limelight3A.class, "ll3a");
        ll3a.setPollRateHz(250);
        ll3a.start();

        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        paths = new Paths(follower);
        Path1 = paths.Path1; Path2 = paths.Path2; Path3 = paths.Path3; Path4 = paths.Path4; Path5 = paths.Path5; Path6 = paths.Path6;
        Path7 = paths.Path7; Path8 = paths.Path8; Path9 = paths.Path9; Path10 = paths.Path10;

        follower.setPose(new Pose(112, 135.600, Math.toRadians(-90)));

        Path1 = paths.Path1; Path2 = paths.Path2; Path3 = paths.Path3; Path4 = paths.Path4;
        Path5 = paths.Path5; Path6 = paths.Path6; Path7 = paths.Path7; Path8 = paths.Path8;
        Path9 = paths.Path9;

        scheduler = CommandScheduler.getInstance(); scheduler.reset(); scheduler = CommandScheduler.getInstance();

        shooter.setHoodAngle(50);

        scheduler.schedule(
                new SequentialCommandGroup(
                        new FraudInstantCommand(()->{
                            targetDeg = -50;
                            intake.setFlap(Intake.flapDown);
                            shooter.setTargetRPM(2700);
                            intake.setPtoEngaged(true);
                        }),
                        new PedroFollowCommand(follower, Path1),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(1200),
                        new IntakeCommand(intake, Intake.flapDown, 1),
                        new FraudInstantCommand(
                                ()->intake.setPtoEngaged(false)
                        ),
                        new PedroFollowCommand(follower, Path2),
                        new PedroFollowCommand(follower, Path3),
                        new FraudInstantCommand(
                                ()->intake.setPtoEngaged(true)
                        ),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(1200),
                        new IntakeCommand(intake, Intake.flapDown, 1),
                        new FraudInstantCommand(
                                ()->intake.setPtoEngaged(false)
                        ),
                        new PedroFollowCommand(follower, Path4),
                        new PedroFollowCommand(follower, Path5),
                        new PedroFollowCommand(follower, Path6),
                        new FraudInstantCommand(
                                ()->intake.setPtoEngaged(true)
                        ),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(1200),
                        new IntakeCommand(intake, Intake.flapDown, 1),
                        new FraudInstantCommand(()->{
                            targetDeg = -135;
                            intake.setPtoEngaged(false);
                        }),
                        new PedroFollowCommand(follower, Path7),
                        new PedroFollowCommand(follower, Path8),
                        new FraudInstantCommand(
                                ()->intake.setPtoEngaged(true)
                        ),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(1200),
                        new IntakeCommand(intake, Intake.flapDown, 1),
                        new IntakeCommand(intake, Intake.flapDown, 0),
                        new PedroFollowCommand(follower, Path9),
                        new WaitCommand(3000),
                        new PedroFollowCommand(follower, Path10)
                )
        );
    }

    @Override
    public void loop() {
        scheduler.run();
        shooter.runShooter();
        follower.update();

        LLResult result = ll3a.getLatestResult();
        boolean detected = result.getBotposeTagCount() > 0;
        boolean locked = false;
        double tX = 0, tY = 0;

        if (detected) {
            for (LLResultTypes.FiducialResult result1 : result.getFiducialResults()) {
                if (result1.getFiducialId() == 24) {
                    locked = true;
                    tX = -result1.getTargetXDegrees();
                    tY = result1.getTargetYDegrees();
                    break;
                } else {
                    locked = false;
                }
            }
        }

        telemetry.addData("locked?", locked);

        if (locked) {
            double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + tY);
            double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
            turret.setPower(p*(0-tX));

            double meters = distanceFromLimelightToGoalInches * 0.0254;
            double turretDeg = analogVoltageToDegrees(encoder.getVoltage());

            shooter.setTargetRPM(getRPMForShot(meters) + c);
            shooter.setHoodAngle(getHoodAngle(meters));
            lastTurretTarget = turretDeg;
        } else {
            double turretDeg = analogVoltageToDegrees(encoder.getVoltage());

            double error = AngleUnit.normalizeDegrees(targetDeg-turretDeg);

            turret.setPower(p * error);
        }
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
        public PathChain Path10;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(112.250, 136.421), new Pose(98.321, 88.080))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(98.321, 88.080),
                                    new Pose(94.634, 80.501),
                                    new Pose(116, 82.754)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(116, 82.754), new Pose(89.104, 82.959))
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(89.104, 82.959),
                                    new Pose(84.188, 55.511),
                                    new Pose(115, 59.812)
                            )
                    )
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(115, 59.812), new Pose(128, 75))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-90))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(128, 75), new Pose(88.899, 83.368))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(0))
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    new Pose(88.899, 83.368),
                                    new Pose(81.320, 29.701),
                                    new Pose(124.541, 35.232)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(124.541, 35.232), new Pose(88.489, 82.754))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-63))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88.489, 82.754), new Pose(124, 74))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(-63), Math.toRadians(0))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(124, 74), new Pose(110, 74))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
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


