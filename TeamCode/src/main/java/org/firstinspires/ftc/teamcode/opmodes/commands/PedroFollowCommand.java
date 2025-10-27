package org.firstinspires.ftc.teamcode.opmodes.commands;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;


public class PedroFollowCommand extends CommandBase {

    private Follower follower;
    private Path path;
    private PathChain pathChain;
    private boolean started = false, isPathChain = false;

    public PedroFollowCommand(Follower follower, Path path) {
        this.follower = follower;
        this.path = path;
    }
    public PedroFollowCommand(Follower follower, PathChain pathChain) {
        this.follower = follower;
        this.pathChain = pathChain;
        isPathChain = true;
    }

    @Override
    public void execute() {
        if (!started && isPathChain) {
            follower.followPath(pathChain);
            started = true;
        } else if (!started && !isPathChain) {
            follower.followPath(path);
            started = true;
        }
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
