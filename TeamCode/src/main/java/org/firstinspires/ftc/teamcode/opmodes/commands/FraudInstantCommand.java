package org.firstinspires.ftc.teamcode.opmodes.commands;

import com.arcrobotics.ftclib.command.CommandBase;

public class FraudInstantCommand extends CommandBase {

    private Runnable runnable;


    public FraudInstantCommand(Runnable runnable) {
        this.runnable = runnable;
    }

    @Override
    public void execute() {
        runnable.run();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
