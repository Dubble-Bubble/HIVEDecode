package org.firstinspires.ftc.teamcode.opmodes.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.systems.pureP.PurePursuitPath;
import org.firstinspires.ftc.teamcode.systems.pureP.PurePursuitSquidFollower;

public class PurePursuitFollowPath extends CommandBase {

    private PurePursuitSquidFollower follower;
    private PurePursuitPath path;
    private boolean started = false, isPathChain = false;

    private double translationalTol = 2, headingTol = 3, timeout = 0.5;

    public PurePursuitFollowPath(PurePursuitSquidFollower follower, PurePursuitPath path, double translationalTol, double headingTol) {
        this.follower = follower;
        this.path = path;
        this.translationalTol = translationalTol;
        this.headingTol = headingTol;
    }

    public PurePursuitFollowPath(PurePursuitSquidFollower follower, PurePursuitPath path, double translationalTol, double headingTol, double timeout) {
        this.follower = follower;
        this.path = path;
        this.translationalTol = translationalTol;
        this.headingTol = headingTol;
        this.timeout = timeout;
    }

    @Override
    public void execute() {
        if (!started) {
            follower.setCurrentPath(path);
            started = true;
            PurePursuitSquidFollower.translationalTolerance = translationalTol; PurePursuitSquidFollower.headingTolerance = headingTol;
            PurePursuitSquidFollower.timeout = timeout;
        }
    }

    @Override
    public boolean isFinished() {
        return started && follower.isFinished();
    }
}
