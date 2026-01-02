package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class HoodTuner extends OpMode {

    Servo leftHood, rightHood;
    public static double pos = 0;


    @Override
    public void init() {
        leftHood = hardwareMap.servo.get("lHood");
        rightHood = hardwareMap.servo.get("rHood");
        leftHood.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        leftHood.setPosition(pos);
        rightHood.setPosition(pos);
    }
}
