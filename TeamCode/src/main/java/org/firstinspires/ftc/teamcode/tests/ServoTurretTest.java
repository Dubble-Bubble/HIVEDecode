package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
@Config
public class ServoTurretTest extends OpMode {

    CRServo servo1, servo2;
    public static double power = 0;
    public static DcMotorSimple.Direction direction = DcMotor.Direction.FORWARD;


    @Override
    public void init() {
        servo1 = hardwareMap.crservo.get("s1");
        servo2 = hardwareMap.crservo.get("s2");
    }

    @Override
    public void loop() {
        servo1.setPower(power);
        servo2.setPower(power);
    }
}
