package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.tests.ShotAlgTest.c;
import static org.firstinspires.ftc.teamcode.tests.ShotAlgTest.f;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
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
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

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
import org.firstinspires.ftc.teamcode.tests.ShotAlgTest;

@Autonomous
public class BlueAutoLimelight extends OpMode {

    Follower follower; Turret turret;

    public static Pose2D endpose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.RADIANS,0);

    Intake intake;

    Shooter shooter;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6, Path7, Path8, Path9;

    Paths paths;

    CommandScheduler scheduler;


    @Override
    public void init() {

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap, telemetry);

        turret = new Turret(hardwareMap, false);
        turret.setMode(Turret.Mode.odo);

        follower.setPose(new Pose(32.5, 135.600, Math.toRadians(-90)));

        paths = new Paths(follower);
        Path1 = paths.Path1; Path2 = paths.Path2; Path3 = paths.Path3; Path4 = paths.Path4;
        Path5 = paths.Path5; Path6 = paths.Path6; Path7 = paths.Path7; Path8 = paths.Path8;
        Path9 = paths.Path9;

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        scheduler = CommandScheduler.getInstance(); scheduler.reset(); scheduler = CommandScheduler.getInstance();
        shooter.setHoodAngle(50);

        scheduler.schedule(
                new SequentialCommandGroup(
                        new FraudInstantCommand(()->{
                            intake.setFlap(Intake.flapUp);
                            intake.setActive(true);
                        }),
                        new PedroFollowCommand(follower, Path1),
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
                        new ParallelCommandGroup(
                                //new PedroFollowCommand(follower, Gurt)
                        ),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new PedroFollowCommand(follower, Path2),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(750),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path3),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path4),
                                new IntakeCommand(intake, Intake.flapUp, 1)

                        ),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(750),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new PedroFollowCommand(follower, Path5),
                                        new PedroFollowCommand(follower, paths.InterPath)
                                ),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new WaitCommand(1200),
                        new IntakeCommand(intake, Intake.flapDown, 1),
                        new PedroFollowCommand(follower, Path6),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapUp, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(true);
                                })
                                ),
                        new WaitCommand(750),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path7)
                        ),
                        new PedroFollowCommand(follower, Path8),
                        new FraudInstantCommand(()->{
                            intake.setTransfer(true);
                        }),
                        new WaitCommand(750),
                        new ParallelCommandGroup(
                                new IntakeCommand(intake, Intake.flapDown, 1),
                                new FraudInstantCommand(()->{
                                    intake.setTransfer(false);
                                })
                        ),
                        new PedroFollowCommand(follower, Path9),
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

    double looptime = 0; boolean turretUpdateFlag = true;
    ElapsedTime looptimer = new ElapsedTime();

    private Pose pose; private double meters;

    @Override
    public void loop() {
        scheduler.run();
        follower.update();
        pose = follower.getPose();

        turret.setPose(new Pair<>(pose.getX(), pose.getY()), Math.toDegrees(pose.getHeading()));

        meters = turret.distanceToGoal(pose.getX(), pose.getY()) * 0.0254;
        shooter.setTargetRPM(shooter.getRPMForShot(meters)+1400);
        shooter.setHoodAngle(shooter.getHoodAngle(meters));

        shooter.runShooter();
        turret.update();
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
                                    new Pose(21, 65)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            InterPath = follower.pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(21, 65), new Pose(10.406, 53))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(145))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(10.406, 53), new Pose(57.797, 70.875))
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
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(57.094, 74.250), new Pose(36.141, 73.688))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
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
