package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class CRServoTuner extends OpMode {

    CRServo servo;
    public static double power = 0;
    public static String name = "name";
    public static DcMotorSimple.Direction direction = DcMotor.Direction.FORWARD;


    @Override
    public void init() {
        servo = hardwareMap.crservo.get(name);
        servo.setDirection(direction);
    }

    @Override
    public void loop() {
        servo.setPower(power);
    }
}
