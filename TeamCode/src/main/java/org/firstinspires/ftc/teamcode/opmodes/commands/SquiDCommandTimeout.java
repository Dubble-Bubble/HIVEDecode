package org.firstinspires.ftc.teamcode.opmodes.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.teamcode.systems.squid.SquIDFollower;

public class SquiDCommandTimeout extends CommandBase {

    private SquIDFollower follower;
    private SparkFunOTOS.Pose2D pose2D;
    private boolean firstRun = false;

    public double ptol, htol, timeout;

    public SquiDCommandTimeout(SquIDFollower follower, double posTolerance, double headingTolerance, SparkFunOTOS.Pose2D pose2D, double timeout) {
        this.follower = follower;
        this.pose2D = pose2D;
        this.ptol = posTolerance;
        this.htol = headingTolerance;
        this.timeout = timeout;
    }

    @Override
    public void execute() {
        follower.setTargetPose(pose2D);
        SquIDFollower.translationalTolerance = ptol;
        SquIDFollower.headingTolerance = htol;
        SquIDFollower.timeout = timeout;
    }

    public boolean isFirstRun = true;

    @Override
    public boolean isFinished() {
        if (isFirstRun) {
            isFirstRun  = false;
            return false;
        } else {
            return follower.isFinished();
        }
    }
}
