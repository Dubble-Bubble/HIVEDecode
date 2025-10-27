package org.firstinspires.ftc.teamcode.opmodes.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.systems.Intake;

public class IntakeCommand extends CommandBase {

    private Intake intake;

    private double intakePower = 0, flapPos = 0.17;

    public IntakeCommand(Intake intake, double flapPos, double intakePower) {
        this.intakePower = intakePower;
        this.intake = intake;
        this.flapPos = flapPos;
    }

    @Override
    public boolean isFinished() {
        intake.setIntake(intakePower);
        intake.setFlap(flapPos);
        return true;
    }
}
