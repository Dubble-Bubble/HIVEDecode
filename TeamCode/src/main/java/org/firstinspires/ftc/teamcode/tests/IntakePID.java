package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
@Config
public class IntakePID extends OpMode {

    public static double zp, zi, zd, zf;
    public static boolean directPower = true;
    public static double topSpeed = 0;
    public static double targetVelocity = 0;
    public static double intakePower = 0;

    private DcMotorEx intake;
    private PIDController controller;

    @Override
    public void init() {
        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller = new PIDController(zp, zi, zd);
    }

    @Override
    public void loop() {
        double intakeSpeed = intake.getVelocity();
        telemetry.addData("intake speed", intakeSpeed);

        controller.setPID(zp, zi, zd);

        if (directPower) {
            intake.setPower(intakePower);
        } else {
            intake.setPower(controller.calculate(intakeSpeed, targetVelocity) + (zf * targetVelocity));
        }
        telemetry.update();
    }
}
