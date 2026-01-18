package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Shooter {

        public static double kP = 0.004, kI, kD, kF = 0.0002;
        private PIDController velController;

        private DcMotorEx shooter, shooter2; private Servo rightHood, leftHood;

        private final double ratio = 32.0/340.0, range = 355;

        private double getServoPosition(double degrees) {
            return ((degrees-75)/ratio)/range;
        }

        private DcMotor turret;
        private Limelight3A ll3a;
        private AnalogInput encoder;

        private PIDController turretPID;

        public static double p, i, d;

        public double targetRPM = 0;

        public void setTargetRPM(double rpm) {
            targetRPM = rpm;
        }

        VoltageSensor voltageSensor;

        Telemetry telemetry;

        public Shooter(HardwareMap hardwareMap, Telemetry t) {
            shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
            shooter2 = (DcMotorEx) hardwareMap.dcMotor.get("shooter2");
            rightHood = hardwareMap.servo.get("rHood");
            leftHood = hardwareMap.servo.get("lHood");

            telemetry = t;

            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
            leftHood.setDirection(Servo.Direction.REVERSE);

            voltageSensor = hardwareMap.voltageSensor.iterator().next();

//            ll3a = hardwareMap.get(Limelight3A.class, "ll3a");

            velController = new PIDController(kP, kI, kD);
        }

        private double targetDeg = 0;

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

    public static double maxTheoreticalRPM = 6461.9047619;

    private double maxRPM = 48.67, rpm = 0, power = 0, scalar = 0;

    ElapsedTime functionRunLength = new ElapsedTime();
    private double runMs = 0;

    public void runShooter() {
        functionRunLength.reset();
        encoderVelocity = -shooter.getVelocity();

        rpm = (encoderVelocity*60)/28;
        readRPM = rpm;

        power = velController.calculate(rpm, targetRPM) + (kF*targetRPM);

        scalar = 13.2/voltageSensor.getVoltage();

        shooter.setPower(power*scalar);
        shooter2.setPower(power*scalar);
        runMs = functionRunLength.milliseconds();
    }

    public double getRunMs() {
        return runMs;
    }

    public double getRPMForShot(double meters) {
//        return (211.43 * meters) + 1177;
            return (227.87*meters) + 1382.7;
        }

        public double getHoodAngle(double meters) {
//          return (-8.8 * meters) + 76.16;
          return Math.max((-4.8701*meters) + 59.754, 46);
        }

        private double encoderVelocity = 0, readRPM = 0, fraudRPM = 0;

    public double getFraudRPM() {
        return fraudRPM;
    }

    public double getEncoderVelocity() {
        return encoderVelocity;
    }

    public double getReadRPM() {
        return readRPM;
    }

    public void update() {
            double turretDeg = analogVoltageToDegrees(encoder.getVoltage());
            turret.setPower(turretPID.calculate(turretDeg, targetDeg));
    }

        public void directSet(double p) {
            shooter.setPower(p);
            shooter2.setPower(p);
        }

        private double analogVoltageToDegrees(double voltage) {
            return voltage * (360/3.3);
        }

        public void stopShooter() {
            shooter.setPower(0);
            shooter2.setPower(0);
        }

        public void setTurretTarget(double t) {
            targetDeg = 0;
        }

}
