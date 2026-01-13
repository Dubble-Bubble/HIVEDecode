package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

@TeleOp
@Config
public class Newshooter extends OpMode {

    private static final Logger log = LoggerFactory.getLogger(Newshooter.class);
    private DcMotorEx shooter, shooter2;
    private PIDController velController;
    private Servo lHood, rHood;
    private DcMotor intake, transfer;
    public static double shooterRpm = 0, intakePower = 0, hoodPosition = 0, p = 0;
    public static double kP = 0.0016, kI, kD, kF = 0.00018;
    public static Servo.Direction rHoodDirection = Servo.Direction.FORWARD, lHoodDirection = Servo.Direction.FORWARD;
    DcMotorSimple.Direction direction1 = DcMotorSimple.Direction.FORWARD, direction2 = DcMotorSimple.Direction.FORWARD;
    VoltageSensor voltageSensor;

    @Override
    public void init() {
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
        shooter2 = (DcMotorEx) hardwareMap.dcMotor.get("shooter2");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();

        shooter.setDirection(direction1);
        shooter2.setDirection(direction2);

        velController = new PIDController(kP, kI, kD);

        intake = hardwareMap.dcMotor.get("intake");
        transfer = hardwareMap.dcMotor.get("transfer");
    }

    public static double signVel = 1;

    @Override
    public void loop() {
        velController.setPID(kP, kI, kD);
        double encoderVelocity = shooter.getVelocity();

        double rpm = (encoderVelocity*60)/28*signVel;
        telemetry.addData("rpm", rpm);
        telemetry.update();

        double power = velController.calculate(rpm, shooterRpm) + (kF*shooterRpm);

        double scalar = 13.0/voltageSensor.getVoltage();

        shooter.setPower(power*scalar);
        shooter2.setPower(power*scalar);
        transfer.setPower(-intakePower);
        intake.setPower(intakePower);
    }
}
