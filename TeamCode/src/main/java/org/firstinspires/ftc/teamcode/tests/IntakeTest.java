package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.systems.Drivebase;

@TeleOp
@Config
public class IntakeTest extends OpMode {

    private DcMotor intake, transfer;
    public static double runPower = 0, transferPower = 0;

    Drivebase drivebase;

    public static DcMotorSimple.Direction intakeDirection = DcMotorSimple.Direction.REVERSE,
            transferDirection = DcMotorSimple.Direction.FORWARD;

    @Override
    public void init() {
        intake = hardwareMap.dcMotor.get("intake");
        transfer = hardwareMap.dcMotor.get("transfer");

        drivebase = new Drivebase(hardwareMap, true);

        intake.setDirection(intakeDirection);
        transfer.setDirection(transferDirection);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transfer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        intake.setPower(gamepad1.right_trigger);
        transfer.setPower(gamepad1.left_trigger);

        drivebase.takeTeleInput(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x);
        drivebase.update();
    }
}
