package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.systems.Shooter;

@TeleOp
@Config
public class ShotAlgTest extends OpMode {

    Shooter shooter;
    public static double zp = 0.001, zi, zd, zf = 0.00067;
    public static boolean directPower = true;
    public static double topSpeed = 0;
    public static double targetVelocity = 0;
    public static double intakePower = 0;

    private DcMotorEx intake;
    private PIDController controller;

    public static double c = 1290, f = 1900;

    public double getRPMForShot(double meters) {
//        return (211.43 * meters) + 1177;
        return (227.87*meters) + 1382.7;
    }

    public double getHoodAngle(double meters) {
//        return (-8.8 * meters) + 76.16;
        return (-4.8701*meters) + 59.754;
    }

    Limelight3A ll3a;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        shooter = new Shooter(hardwareMap, telemetry);
        ll3a = hardwareMap.get(Limelight3A.class, "ll3a");
        ll3a.setPollRateHz(250);
        ll3a.start();

        intake = (DcMotorEx) hardwareMap.dcMotor.get("intake");
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controller = new PIDController(zp, zi, zd);
    }

    double limelightMountAngleDegrees = 10.0, limelightLensHeightInches = 13.4, goalHeightInches = 29.5;

    public static double ip = 0;

    @Override
    public void loop() {
        LLResult result = ll3a.getLatestResult();

        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + result.getTy());

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        double meters = distanceFromLimelightToGoalInches * 0.0254;

        double targetRPM = getRPMForShot(meters) + c;
        double hoodangle = getHoodAngle(meters);

        telemetry.addData("distance in inches", distanceFromLimelightToGoalInches);
        telemetry.addData("meters", meters);
        telemetry.addData("targetRPM", targetRPM);
        telemetry.addData("targetHood", hoodangle);
        telemetry.addData("Encoder TPS", shooter.getEncoderVelocity());
        telemetry.addData("Encoder TPS to RPM", (shooter.getEncoderVelocity()*60)/28);

        shooter.setTargetRPM(targetRPM);
        shooter.setHoodAngle(hoodangle);
        shooter.runShooter();

        double intakeSpeed = intake.getVelocity();
        telemetry.addData("intake speed", intakeSpeed);

        controller.setPID(zp, zi, zd);

        if (directPower) {
            intake.setPower(intakePower);
        } else {
            intake.setPower(controller.calculate(intakeSpeed, targetVelocity) + (zf * targetVelocity));
        }
        telemetry.update();
    }
}
