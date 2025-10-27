package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.systems.Shooter;

@Config
@TeleOp
public class ShooterTest extends OpMode {

    private DcMotorEx shooter;

    Servo leftHood, rightHood;

    private PIDController shooterPID;

    public static double p, i, d, f;

    public static double targetRPM = 6000;
    public static boolean runShooter = false;

    public static double hoodAnlge = 75;

    ElapsedTime runtime = new ElapsedTime();

    private DcMotor intake;

    @Override
    public void init() {
        shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        shooterPID = new PIDController(p, i, d);

        leftHood = hardwareMap.servo.get("lHood");
        rightHood = hardwareMap.servo.get("rHood");
        leftHood.setDirection(Servo.Direction.REVERSE);

        intake = hardwareMap.dcMotor.get("intake");

    }

    private double maxRPM = 48.67;

    @Override
    public void loop() {

        shooterPID.setPID(p, i, d);

        double v = shooter.getVelocity(AngleUnit.RADIANS);

        double fRMP = ((v*60)/(2*Math.PI));
        double rpm = 6521*(fRMP/maxRPM);
        setServos(getServoPosition(hoodAnlge));

        if (runShooter) {
            shooter.setPower(shooterPID.calculate(rpm, targetRPM) + (f*targetRPM));
        }

        intake.setPower(gamepad1.right_trigger);

        telemetry.addData("Velocity, Degrees/S", shooter.getVelocity(AngleUnit.DEGREES));
        telemetry.addData("RPM", rpm);
        telemetry.addData("Target RPM", targetRPM);
        telemetry.update();
    }

    public void setHoodAngle(double degrees) {
        if (degrees >= 75) {
            setServos(0);
        } else {
            setServos(Math.abs(getServoPosition(degrees)));
        }
    }

    private void setServos(double pos) {
        leftHood.setPosition(pos);
        rightHood.setPosition(pos);
    }

    private final double ratio = 32.0/340.0, range = 355;

    private double getServoPosition(double degrees) {
        return ((75-degrees)/ratio)/range;
    }
}
