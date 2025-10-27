package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class FlapTester extends OpMode {
    public static double pos = 0;
    private Servo flapL, flapR;


    @Override
    public void init() {
        flapL = hardwareMap.servo.get("lFlap");
        flapR = hardwareMap.servo.get("rFlap");
        flapR.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void loop() {
        flapR.setPosition(pos);
        flapL.setPosition(pos);
    }
}
