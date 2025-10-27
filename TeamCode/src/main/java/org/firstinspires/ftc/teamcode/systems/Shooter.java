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

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Shooter {

        public static double kP, kI, kD, kF;
        private PIDController velController;

        private DcMotorEx shooter; private Servo rightHood, leftHood;

        private final double ratio = 32.0/340.0, range = 355;

        private double getServoPosition(double degrees) {
            return ((degrees-75)/ratio)/range;
        }

        private DcMotor turret;
        private Limelight3A ll3a;
        private AnalogInput encoder;

        private PIDController turretPID;

        public static double p, i, d;

        public static double targetRPM = 6000;

        public Shooter(HardwareMap hardwareMap) {
            shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
            rightHood = hardwareMap.servo.get("rHood");
            leftHood = hardwareMap.servo.get("lHood");

            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setDirection(DcMotorSimple.Direction.REVERSE);
            leftHood.setDirection(Servo.Direction.REVERSE);

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

        public void runShooter() {
            double vel = shooter.getVelocity(AngleUnit.RADIANS) * 60;
            velController.setPID(kP, kI, kD);
            shooter.setPower(velController.calculate(vel, targetRPM) + (kF * targetRPM));
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

        public void setTurretTarget(double t) {
            targetDeg = 0;
        }

}
