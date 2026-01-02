package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp
@Config
public class MotorTuner extends OpMode {

    private DcMotorEx motor;

    public static double power = 0;
    public static String name = "name";
    public static DcMotor.Direction direction = DcMotor.Direction.FORWARD;


    @Override
    public void init() {
        motor = (DcMotorEx) hardwareMap.dcMotor.get(name);
        motor.setDirection(direction);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        motor.setPower(power);
        telemetry.addData("current", motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("TPS", motor.getVelocity());
        telemetry.addData("435 TPS: ", (435/60)*384.5);
        telemetry.addData("312 TPS: ", (312/60)*537.7);
        telemetry.update();
    }

}
