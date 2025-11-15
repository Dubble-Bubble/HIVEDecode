package org.firstinspires.ftc.teamcode.opmodes.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.robocol.Command;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Shooter;

public class ManualShooterInputCommand extends CommandBase {

    private Shooter shooter;

    private ElapsedTime elapsedTime = new ElapsedTime();

    private double targetRPM = 0, runtime = 0, hoodAngle = 0;

    public ManualShooterInputCommand(Shooter shooter, double rpmInput, double timeToRun, double hoodAngle) {
        this.shooter = shooter;
        this.targetRPM = rpmInput;
        this.runtime = timeToRun;
        this.hoodAngle = hoodAngle;
    }

    private boolean first = false;

    @Override
    public void execute() {
        if (!first) {
            first = true;
            shooter.setTargetRPM(targetRPM);
            elapsedTime.reset();
            shooter.setHoodAngle(hoodAngle);
        }
        shooter.runShooter();
    }

    @Override
    public boolean isFinished() {
        return elapsedTime.seconds() >= runtime;
    }
}
