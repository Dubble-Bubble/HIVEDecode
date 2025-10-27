package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
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

    private DcMotor turret;
    private AnalogInput encoder;

    private PIDController turretPID;

    public static double p, i, d;

    Limelight3A ll3a;

    @Override
    public void init() {
        turret = hardwareMap.dcMotor.get("turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        ll3a = hardwareMap.get(Limelight3A.class, "ll3a");
        ll3a.start();

        turretPID = new PIDController(p, i, d);

//        encoder = hardwareMap.analogInput.get("encoder");

        turretPID.setSetPoint(0);
    }

    double limelightMountAngleDegrees = 5.0, limelightLensHeightInches = 13.4, goalHeightInches = 60.0;

    @Override
    public void loop() {

        turretPID.setPID(p, i, d);

        LLResult result = ll3a.getLatestResult();

        double tX = -result.getTx();

        double angleToGoalRadians = Math.toRadians(limelightMountAngleDegrees + result.getTy());

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

        turret.setPower(turretPID.calculate(tX));

        telemetry.addData("Angle Error", -tX);
        telemetry.addData("Distance to tag", distanceFromLimelightToGoalInches);
        telemetry.update();
    }
}
