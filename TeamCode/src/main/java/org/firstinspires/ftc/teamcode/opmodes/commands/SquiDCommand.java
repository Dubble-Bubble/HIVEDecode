package org.firstinspires.ftc.teamcode.opmodes.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.systems.squid.SquIDFollower;

public class SquiDCommand extends CommandBase {

    private SquIDFollower follower;
    private SparkFunOTOS.Pose2D pose2D;
    private boolean firstRun = false;

    public SquiDCommand(SquIDFollower follower, double posTolerance, double headingTolerance, SparkFunOTOS.Pose2D pose2D) {
        this.follower = follower;
        SquIDFollower.translationalTolerance = posTolerance;
        SquIDFollower.headingTolerance = headingTolerance;
        this.pose2D = pose2D;
    }

    @Override
    public void execute() {
        follower.setTargetPose(pose2D);
    }

    @Override
    public boolean isFinished() {
        return follower.isFinished();
    }
}
