package org.firstinspires.ftc.teamcode.opmodes.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.systems.Shooter;

public class SpinUpShooter extends CommandBase {
        private Shooter shooter;

        ElapsedTime elapsedTime = new ElapsedTime();
        boolean first;

        public SpinUpShooter(Shooter shooter) {
            this.shooter = shooter;
        }

        @Override
        public void execute() {
            elapsedTime.reset();
        }

        @Override
        public boolean isFinished() {
            return shooter.atTarget() && elapsedTime.seconds() > 0.2;
        }
}
