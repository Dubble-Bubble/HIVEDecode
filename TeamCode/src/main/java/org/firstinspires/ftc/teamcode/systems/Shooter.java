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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Shooter {

        public static double kP = 0.001, kI, kD, kF = 0.00016;
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
            shooter.setDirection(DcMotorSimple.Direction.REVERSE);
            leftHood.setDirection(Servo.Direction.REVERSE);

            voltageSensor = hardwareMap.voltageSensor.iterator().next();

            turret = hardwareMap.dcMotor.get("turret");
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

//            ll3a = hardwareMap.get(Limelight3A.class, "ll3a");

            encoder = hardwareMap.analogInput.get("encoder");

            turretPID = new PIDController(p, i, d);

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

    private double maxRPM = 48.67;

        public void runShooter() {
            double v = -shooter.getVelocity(AngleUnit.RADIANS);

            double fRMP = ((v*60)/(2*Math.PI));
            double rpm = 6521.7*(fRMP/maxRPM);

            double power = velController.calculate(rpm, targetRPM) + (kF*targetRPM);

            double scalar = 13.0/voltageSensor.getVoltage();

            shooter.setPower(power*scalar);
            shooter2.setPower(power*scalar);

        }

        public void update() {
            double turretDeg = analogVoltageToDegrees(encoder.getVoltage());
            turret.setPower(turretPID.calculate(turretDeg, targetDeg));
        }

        public void directSet(double p) {
            shooter.setPower(p);
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
