package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class MotorTuner extends OpMode {

    private DcMotor motor;

    public static double power = 0;
    public static String name = "name";
    public static DcMotor.Direction direction = DcMotor.Direction.FORWARD;


    @Override
    public void init() {
        motor = hardwareMap.dcMotor.get(name);
        motor.setDirection(direction);
    }

    @Override
    public void loop() {
        motor.setPower(power);
    }

}
