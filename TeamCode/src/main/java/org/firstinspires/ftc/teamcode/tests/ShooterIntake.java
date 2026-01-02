package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.systems.Shooter;

@Config
@TeleOp
public class ShooterIntake extends OpMode {

    private Shooter shooter;
    public static double hoodAngle = 75;
    public static double velTarget = 0;
    public static boolean runShooter = false;

    private DcMotor intake;
    private DcMotor transfer;

    public static double intakeP = 0, transferP = 0;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap, telemetry);
        intake = hardwareMap.dcMotor.get("intake");
        transfer = hardwareMap.dcMotor.get("transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        if (runShooter) {
            shooter.setTargetRPM(velTarget);
            shooter.runShooter();
        }

        telemetry.addData("targetRPM", velTarget);
        telemetry.addData("RPM", (shooter.getEncoderVelocity()*60)/28);
        telemetry.update();

        shooter.setHoodAngle(hoodAngle);
        intake.setPower(intakeP);
        transfer.setPower(intakeP);
    }
}
