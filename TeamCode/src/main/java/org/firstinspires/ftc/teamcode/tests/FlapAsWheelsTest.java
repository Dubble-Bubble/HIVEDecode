package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class FlapAsWheelsTest extends OpMode {
    public static double pos = 0, ip = 0;
    private CRServo flapL, flapR;

    private DcMotor intake;


    @Override
    public void init() {
        flapL = hardwareMap.crservo.get("lFlap");
        flapR = hardwareMap.crservo.get("rFlap");
        flapR.setDirection(CRServo.Direction.REVERSE);

        intake = hardwareMap.dcMotor.get("intake");
    }

    @Override
    public void loop() {
        flapR.setPower(pos);
        flapL.setPower(pos);
        intake.setPower(ip);
    }
}