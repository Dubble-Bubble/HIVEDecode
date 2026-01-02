package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.systems.Localizer;

@TeleOp
@Config
public class TurretPositionTracking extends OpMode {

    private CRServo s1, s2;
    private AnalogInput encoder;

    private PIDController turretPID;

    public static double targetDeg = 0;

    public static double p = 0.03, i, d, f, b;

    public static boolean runTurret = false;

    Limelight3A ll3a;

    @Override
    public void init() {
        s1 = hardwareMap.crservo.get("cts");
        s2 = hardwareMap.crservo.get("mts");

        turretPID = new PIDController(p, i, d);

        encoder = hardwareMap.analogInput.get("encoder");

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
//

    }

    double limelightMountAngleDegrees = 5.0, limelightLensHeightInches = 13.4, goalHeightInches = 29.5;

    @Override
    public void loop() {

        turretPID.setPID(p, i, d);
//
//        LLResult result = ll3a.getLatestResult();
//
//        double tX = -result.getTx();
//
//        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + result.getTy());
//
//        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        double turretDeg = analogVoltageToDegrees(encoder.getVoltage());



        telemetry.update();
    }

    private double analogVoltageToDegrees(double voltage) {
        return voltage * (360/3.3);
    }

    private void setPower(double power) {
        s1.setPower(power);
        s2.setPower(power);
    }

    public double getFitP(double error) {
        return  Math.max(0.556 + (-0.086*Math.log(error)), 0.03);
    }
}
