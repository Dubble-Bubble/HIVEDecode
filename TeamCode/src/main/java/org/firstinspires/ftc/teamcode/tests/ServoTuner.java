package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class ServoTuner extends OpMode {

    Servo servo;
    public static double pos = 0;
    public static String name = "name";
    public static Servo.Direction direction = Servo.Direction.FORWARD;


    @Override
    public void init() {
        servo = hardwareMap.servo.get(name);
        servo.setDirection(direction);
    }

    @Override
    public void loop() {
        servo.setPosition(pos);
    }
}
