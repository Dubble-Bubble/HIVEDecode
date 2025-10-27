package org.firstinspires.ftc.teamcode.opmodes;

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

import org.firstinspires.ftc.teamcode.opmodes.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.opmodes.commands.PedroFollowCommand;
import org.firstinspires.ftc.teamcode.pedropathing.Constants;
import org.firstinspires.ftc.teamcode.systems.Intake;

@Autonomous
public class RedAuto extends OpMode {

    Follower follower;

    Intake intake;

    public PathChain Path1, Path2, Path3, Path4, Path5, Path6;

    CommandScheduler scheduler;

    @Override
    public void init() {

        follower = Constants.createFollower(hardwareMap);
        intake = new Intake(hardwareMap);

        follower.setPose(new Pose(87.800, 8.000, Math.toRadians(90)));

        Path1 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(87.800, 8.000),
                                new Pose(72.844, 35.719),
                                new Pose(124.734, 35.438)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        Path2 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(124.734, 35.438), new Pose(85.078, 13.078))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                .build();

        Path3 = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(85.078, 13.078),
                                new Pose(88.734, 60.750),
                                new Pose(125.719, 59.344)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();

        Path4 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(125.719, 59.344), new Pose(82.688, 81.563))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path5 = follower.pathBuilder().addPath(
                        new BezierLine(new Pose(82.688, 81.563), new Pose(125.859, 83.813))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(125.859, 83.813), new Pose(81.984, 84.234))
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .setReversed()
                .build();

        scheduler = CommandScheduler.getInstance(); scheduler.reset(); scheduler = CommandScheduler.getInstance();

        scheduler.schedule(
                new SequentialCommandGroup(
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(2000),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path1),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new WaitCommand(300),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path2),
                                new IntakeCommand(intake, Intake.flapDown, 0)
                        ),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(2000),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path3),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new WaitCommand(300),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path4),
                                new IntakeCommand(intake, Intake.flapDown, 0)
                        ),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(2000),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path5),
                                new IntakeCommand(intake, Intake.flapDown, 1)
                        ),
                        new WaitCommand(300),
                        new ParallelCommandGroup(
                                new PedroFollowCommand(follower, Path6),
                                new IntakeCommand(intake, Intake.flapDown, 0)
                        ),
                        new IntakeCommand(intake, Intake.flapUp, 1),
                        new WaitCommand(2000)
                )
        );

        intake.setFlap(Intake.flapDown);

    }

    @Override
    public void loop() {
        scheduler.run();
        follower.update();
    }
}
