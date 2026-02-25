package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@TeleOp
@Config
public class ArbitraryShooterTest extends OpMode {

    private DcMotorEx shooter, shooter2;
    private Servo hood;
    public static double shooterRpm = 0, shooterDirectPower = 0, hoodPosition = 0;
    public static double kP = 0, kV = 0, gearRatio = 1;
    public static DcMotorSimple.Direction shooterMotorDirection = DcMotorSimple.Direction.FORWARD,
    shooterMotor2Direction = DcMotorSimple.Direction.REVERSE;

    public static String shooterMotorName = "shooter", shooterMotor2Name = "shooter2";

    public static boolean runWithPID = false;

    VoltageSensor voltageSensor;

    @Override
    public void init() {

        shooter = (DcMotorEx) hardwareMap.dcMotor.get(shooterMotorName);
        shooter2 = (DcMotorEx) hardwareMap.dcMotor.get(shooterMotor2Name);

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooter.setDirection(shooterMotorDirection);
        shooter2.setDirection(shooterMotor2Direction);

        telemetry = FtcDashboard.getInstance().getTelemetry();
    }

    public static double signVel = 1;

    @Override
    public void loop() {
        double encoderVelocity = shooter.getVelocity();

        double rpm = (encoderVelocity*60)/28*signVel*gearRatio;
        telemetry.addData("rpm", rpm);
        telemetry.update();

        if (runWithPID) {
            if  (rpm < shooterRpm) {
                shooter.setPower(1);
                shooter2.setPower(1);
            } else {
                shooter.setPower(0);
                shooter2.setPower(0);
            }
        } else {
            shooter.setPower(shooterDirectPower);
            shooter2.setPower(shooterDirectPower);
        }

        telemetry.addData("rpm", rpm);
    }
}
