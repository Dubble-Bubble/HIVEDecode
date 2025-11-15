package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;

public class ShotTester extends OpMode {

    private Shooter shooter;
    private Intake intake;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap, telemetry);
        intake = new Intake(hardwareMap);
    }

    @Override
    public void loop() {
        shooter.directSet(1);
        intake.setActive(gamepad1.right_trigger > 0.5);

        shooter.runShooter();
        intake.update();
    }
}
