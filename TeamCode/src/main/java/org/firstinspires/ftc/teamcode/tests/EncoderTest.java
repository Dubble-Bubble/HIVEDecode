package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
@Config
public class EncoderTest extends OpMode {

    private AnalogInput encoder;
    private DcMotor turret;

    private PIDController turretPID;

    public static double p, i, d;

    public static double target = 0;

    @Override
    public void init() {
        encoder = hardwareMap.analogInput.get("encoder");
        turret = hardwareMap.dcMotor.get("turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretPID = new PIDController(p, i, d);
    }

    ElapsedTime runtime = new ElapsedTime();

    private double lastError = 0;

    @Override
    public void loop() {
        runtime.reset();
        double turretDeg = encoder.getVoltage() * (360/3.3);
        telemetry.addData("Read Angle by Encoder", turretDeg);

        turretPID.setPID(p, i, d);

        double error = AngleUnit.normalizeDegrees(target-turretDeg);
        double integral = (error) * runtime.seconds();

        turret.setPower((p*error) + (integral * i));

        runtime.reset();

        telemetry.addData("Encoder Voltage", encoder.getVoltage());
        telemetry.addData("Target", target);
        telemetry.update();
        lastError = error;
    }
}