package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.Intake;

@Config
@TeleOp
public class ptotest extends OpMode {
    public static double ip = 0;
    private Servo pto;
    private Servo pto2;

    private DcMotor intake;

    public static double ptoPos = 0, pto2Pos = 0;

    public static boolean ptoEngaged = false;


    @Override
    public void init() {
        pto = hardwareMap.servo.get("pto");
        pto.setDirection(Servo.Direction.REVERSE);
        pto2 = hardwareMap.servo.get("pto2");

        intake = hardwareMap.dcMotor.get("intake");
    }

    @Override
    public void loop() {

        pto.setPosition(ptoPos);
        pto2.setPosition(pto2Pos);

        intake.setPower(ip);
    }
}