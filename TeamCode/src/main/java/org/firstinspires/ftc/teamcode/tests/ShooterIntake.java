package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.systems.Shooter;

@Config
@TeleOp
public class ShooterIntake extends OpMode {

    private Shooter shooter;
    public static double hoodAngle = 75;
    public static double p = 0;
    public static boolean runShooter = false;

    private DcMotor intake;

    public static double intakeP = 0;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap);
        intake = hardwareMap.dcMotor.get("intake");
    }

    @Override
    public void loop() {
        if (runShooter) {
            shooter.directSet(p);
        }

        shooter.setHoodAngle(hoodAngle);
        intake.setPower(intakeP);
    }
}
