package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.systems.Shooter;

@TeleOp
@Config
public class ShotAlgTest extends OpMode {

    Shooter shooter;
    DcMotor intake;

    public static double c = 2400;

    public double getRPMForShot(double meters) {
        return (211.43 * meters) + 1177;
    }

    public double getHoodAngle(double meters) {
        return (-8.8 * meters) + 76.16;
    }

    Limelight3A ll3a;

    @Override
    public void init() {
        shooter = new Shooter(hardwareMap, telemetry);
        ll3a = hardwareMap.get(Limelight3A.class, "ll3a");
        ll3a.setPollRateHz(250);
        ll3a.start();

        intake = hardwareMap.dcMotor.get("intake");
    }

    double limelightMountAngleDegrees = 10.0, limelightLensHeightInches = 13.4, goalHeightInches = 29.5;

    @Override
    public void loop() {
        LLResult result = ll3a.getLatestResult();

        double tX = -result.getTx();

        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + result.getTy());

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        double meters = distanceFromLimelightToGoalInches * 0.0254;

        telemetry.addData("distance in inches", distanceFromLimelightToGoalInches);
        telemetry.addData("meters", meters);
        telemetry.update();

        shooter.setTargetRPM(getRPMForShot(meters) + c);
        shooter.setHoodAngle(getHoodAngle(meters));
        shooter.runShooter();

        intake.setPower(gamepad1.right_trigger);
    }
}
