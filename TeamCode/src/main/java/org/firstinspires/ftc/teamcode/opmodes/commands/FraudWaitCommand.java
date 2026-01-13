package org.firstinspires.ftc.teamcode.opmodes.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FraudWaitCommand extends CommandBase {

    private ElapsedTime elapsedTime;
    private double ms = 0; private boolean fLoop = true;


    public FraudWaitCommand(double ms) {
        this.ms = ms;
        elapsedTime = new ElapsedTime();
    }

    @Override
    public void execute() {
        if (fLoop) {
            fLoop = false;
            elapsedTime.reset();
        }
    }

    @Override
    public boolean isFinished() {
        if (!fLoop) {
            return elapsedTime.milliseconds() > ms;
        } else {
            return false;
        }
    }
}
