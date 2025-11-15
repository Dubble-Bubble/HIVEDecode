package org.firstinspires.ftc.teamcode.opmodes.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Shooter;

public class StopShooter extends CommandBase {

    private Shooter shooter;
    public StopShooter(Shooter shooter) {
        this.shooter = shooter;

    }

    @Override
    public boolean isFinished() {
        shooter.stopShooter();
        return true;
    }
}
