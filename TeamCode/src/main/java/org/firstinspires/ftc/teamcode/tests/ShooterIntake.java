package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.systems.Shooter;

@Config
@TeleOp
public class ShooterIntake extends OpMode {

    public static double hoodAngle = 75, p, f = 0.000185;
    public static double velTarget = 0;
    public static boolean runShooterPID = false, runShooterBangBang = false;

    private DcMotorEx intake, shooter, shooter2;
    private DcMotor transfer;

    private Servo rightHood, leftHood;

    public static double intakeP = 0, transferP = 0;

    public static DcMotorSimple.Direction shooterDirection = DcMotorSimple.Direction.FORWARD,
            shooter2Direction = DcMotorSimple.Direction.REVERSE;

    @Override
    public void init() {
        shooter =(DcMotorEx) hardwareMap.dcMotor.get("shooter");
        shooter2 = (DcMotorEx) hardwareMap.dcMotor.get("shooter2");

        rightHood = hardwareMap.servo.get("rHood");
        leftHood = hardwareMap.servo.get("lHood");

        leftHood.setDirection(Servo.Direction.REVERSE);

        shooter.setDirection(shooterDirection);
        shooter2.setDirection(shooter2Direction);

        shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        intake =(DcMotorEx) hardwareMap.dcMotor.get("intake");
        transfer = hardwareMap.dcMotor.get("transfer");
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        double vel = shooter.getVelocity();
        vel = ((vel * 60) / 28);

        if (runShooterPID) {
            double power = p * (velTarget-vel) + f * velTarget;
            shooter2.setPower(power);
            shooter.setPower(power);
        }
        if (runShooterBangBang) {
            if (vel < velTarget - 50) {
                shooter2.setPower(1);
                shooter.setPower(1);
            } else {
                shooter2.setPower(f*velTarget);
                shooter.setPower(f * velTarget);
            }
        }

        telemetry.addData("targetRPM", velTarget);
        telemetry.addData("true velocity", vel);

        telemetry.update();

        leftHood.setPosition(Shooter.getLinkageHoodServoPos(hoodAngle));
        rightHood.setPosition(Shooter.getLinkageHoodServoPos(hoodAngle));

        intake.setPower(intakeP);
        transfer.setPower(intakeP);
    }

    private int lastEncoderPos = 0; private long lastEncoderTimeNs = 0;

    public double getBulkVelocity(int pos) {
        int currentPos = pos; // bulk cached
        long currentTimeNs = System.nanoTime();

        double velocity = (currentPos - lastEncoderPos) /
                ((currentTimeNs - lastEncoderTimeNs) / 1e9);

        lastEncoderPos = currentPos;
        lastEncoderTimeNs = currentTimeNs;
        return velocity;
    }
}
