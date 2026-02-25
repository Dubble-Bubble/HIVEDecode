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

        public static double kP = 0.001, kI, kD, kF = 0.00015;
        private PIDController velController;

        private DcMotorEx shooter, shooter2; private Servo rightHood, leftHood;

        private final double ratio = 18.0/168.0, range = 355;

        private double getServoPosition(double degrees) {
            return ((72-degrees)/ratio)/range;
        }

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

            velController = new PIDController(kP, kI, kD);
        }

        private double targetDeg = 0, hoodSet;

        public void setHoodAngle(double degrees) {
            if (!Double.isNaN(degrees)) {
                if (degrees >= 72) {
                    setServos(0.12);
                } else if (!Double.isNaN(Math.abs(getServoPosition(degrees)))){
                    setServos(Math.min(Math.max(Math.abs(getServoPosition(degrees)), 0.12), 0.9));
                }
            }
        }

        private void setServos(double pos) {
            leftHood.setPosition(pos);
            rightHood.setPosition(pos);
        }

    private double maxRPM = 48.67, rpm = 0, power = 0, scalar = 0;

    ElapsedTime functionRunLength = new ElapsedTime();
    private double runMs = 0;
    private double cachedPower = 0;

    @Deprecated
    public void runShooter() {
        functionRunLength.reset();
        encoderVelocity = -shooter.getVelocity();

        rpm = (encoderVelocity*60)/28 * (55/65);
        readRPM = rpm;

        power = velController.calculate(rpm, targetRPM) + (kF*targetRPM);

        scalar = 13.2/voltageSensor.getVoltage();

        shooter.setPower(power*scalar);
        shooter2.setPower(power*scalar);
        runMs = functionRunLength.milliseconds();
    }

    public void runShooterSus() {
        functionRunLength.reset();
        encoderVelocity = -shooter.getVelocity();

        rpm = ((encoderVelocity*60)/28) * 55/65;
        readRPM = rpm;

        if  (rpm < targetRPM) {
            shooter.setPower(1);
            shooter2.setPower(1);
        } else {
            shooter.setPower(0);
            shooter2.setPower(0);
        }
        runMs = functionRunLength.milliseconds();
    }

    public double getRunMs() {
        return runMs;
    }

    @Deprecated
    public double getRPMForShot(double meters) {
//        return (211.43 * meters) + 1177;
            return (227.87*meters) + 1382.7;
    }

    @Deprecated
    public double getHoodAngle(double meters) {
//          return (-8.8 * meters) + 76.16;
        return Math.max((-4.8701*meters) + 59.754, 46);
    }
    private double encoderVelocity = 0, readRPM = 0, fraudRPM = 0;

    public double getEncoderVelocity() {
        return encoderVelocity;
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
    private double a = 0, b = 0, c = 0, n = 0, t_u = 0, t_g = 0,
            tof = 0, vX = 0, vY = 0, v = 0, m = 0;
    private double kinematicRPMGoal = 0;
    public static double w = 1.2;

    public double shooterVKinematic() {
        return v;
    }

    public void updateFancyKinematics(double distMeters, double hoodAngleRad) {
        a = (-distMeters * Math.tan(hoodAngleRad) + w) / (distMeters * distMeters);
        b = -Math.tan(hoodAngleRad)-(2*a*distMeters);
        n = -b/(2*a);
        m = (a * (n*n)) + (b * (n)) + w;

        t_u = Math.sqrt((2*m) / 9.8);
        t_g = Math.sqrt((2*(m-w)) / 9.8);
        tof = t_u + t_g;

        vX = distMeters/tof;
        vY = (m - 0.5*(-9.8)*(t_u*t_u))/t_u;

        v = Math.sqrt((vX*vX) + (vY*vY));

        kinematicRPMGoal = (v / (2*Math.PI * 0.036)) * 60;
    }

    public double vMSToRPM(double vMS) {
        return (vMS / (2*Math.PI * 0.036)) * 60;
    }

    public double getKinematicRPMGoal() {
        return kinematicRPMGoal;
    }

    public double getTof() {
        return tof;
    }
}
