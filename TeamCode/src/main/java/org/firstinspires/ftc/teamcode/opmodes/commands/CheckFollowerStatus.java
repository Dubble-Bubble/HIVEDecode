package org.firstinspires.ftc.teamcode.opmodes.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.sun.tools.javac.comp.Check;

import org.firstinspires.ftc.teamcode.systems.squid.SquIDFollower;

public class CheckFollowerStatus extends CommandBase {

    SquIDFollower follower;

    public CheckFollowerStatus(SquIDFollower follower) {
        this.follower = follower;
    }

    @Override
    public boolean isFinished() {
        return follower.isFinished();
    }

}
