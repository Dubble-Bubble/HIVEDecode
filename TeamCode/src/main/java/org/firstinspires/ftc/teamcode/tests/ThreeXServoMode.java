package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.systems.Turret;

@Config
@TeleOp
public class ThreeXServoMode extends OpMode {

    private Servo s1, s2, s3;

    public static double angleDeg = 0;
    public static double zeroOffset = 1;

    AnalogInput encoder;

    @Override
    public void init() {
        s1 = hardwareMap.servo.get("s1");
        s2 = hardwareMap.servo.get("s2");
        s3 = hardwareMap.servo.get("s3");
    }

    private double servoUnitsPerDegree = 1.0 / 355.0;

    @Override
    public void loop() {

        double inputAngle = AngleUnit.normalizeDegrees(angleDeg) + zeroOffset;

        telemetry.addData("pre-clipped input angle", inputAngle);

        inputAngle = Range.clip(inputAngle, -170 + zeroOffset, 170 + zeroOffset);

        telemetry.addData("post-clipped input angle", inputAngle);

        double initialPos = servoUnitsPerDegree * inputAngle;

        telemetry.addData("servo units per degree", servoUnitsPerDegree);
        telemetry.addData("intialPos", initialPos);

        double pos = 0.5 + initialPos;

        telemetry.addData("pos", pos);

        telemetry.update();

        s1.setPosition(pos);
        s2.setPosition(pos);
        s3.setPosition(pos);
    }
}
