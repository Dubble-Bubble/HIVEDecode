package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.Turret;

@Config
@TeleOp
public class ThreeXServoMode extends OpMode {

    Turret turret;

    public static double angle = 0;

    AnalogInput encoder;

    @Override
    public void init() {
        turret = new Turret(hardwareMap, false);
        turret.setMode(Turret.Mode.fixed);
    }

    @Override
    public void loop() {
        turret.setTargetDegrees(angle);
        turret.update();
        telemetry.addData("angle", turret.getTurretAngle());
        telemetry.update();
    }
}
