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
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
public class Shooter {

    public static double kP = 0.002, kI, kD, kF = 0.000185, velMult = 1;
    private PIDController velController;

    private DcMotorEx shooter, shooter2; private Servo rightHood, leftHood;

    VoltageSensor voltageSensor;

    private final double ratio = 18.0/168.0, range = 355;

    private double getServoPosition(double degrees) {
        return ((72-degrees)/ratio)/range;
    }

    public static DcMotorSimple.Direction s1Direction = DcMotorSimple.Direction.FORWARD, s2Direction = DcMotorSimple.Direction.REVERSE;

    public static double p, i, d;

    public double targetRPM = 0;

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    Telemetry telemetry;

    public static double hoodCacheTolerance = 0.002;
    private double cachedHoodPos = Double.NaN;

    public static double motorPowerCacheTolerance = 0.01;
    private double cachedMotorPower = Double.NaN;

    public void invalidateCaches() {
        cachedMotorPower = Double.NaN;
        cachedHoodPos = Double.NaN;
    }

        public Shooter(HardwareMap hardwareMap, Telemetry t) {
            shooter = (DcMotorEx) hardwareMap.dcMotor.get("shooter");
            shooter2 = (DcMotorEx) hardwareMap.dcMotor.get("shooter2");
            rightHood = hardwareMap.servo.get("rHood");
            leftHood = hardwareMap.servo.get("lHood");

            telemetry = t;

            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setDirection(s1Direction);
            shooter2.setDirection(s2Direction);
            leftHood.setDirection(Servo.Direction.REVERSE);

            velController = new PIDController(kP, kI, kD);

            lastEncoderPos = shooter.getCurrentPosition();
            lastEncoderTimeNs = System.nanoTime();

            voltageSensor = hardwareMap.voltageSensor.iterator().next();
        }

        private double targetDeg = 0, hoodSet, posTemp = 0;

        public void setHoodAngle(double degrees) {
            if (Double.isNaN(degrees)) return;

            posTemp = getLinkageHoodServoPos(degrees);

//            if (Double.isNaN(cachedHoodPos) || Math.abs(posTemp - cachedHoodPos) > hoodCacheTolerance) {
                setServos(posTemp);
//                cachedHoodPos = posTemp;
//            }
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

        shooter.setPower(power*scalar);
        shooter2.setPower(power*scalar);
        runMs = functionRunLength.milliseconds();
    }

    public boolean atTarget() {
        return rpm >= kinematicRPMGoal - 50;
    }

    double newPower = 0;

    public double getRpm() {
        return rpm;
    }


    public static double bangBangRPMDropThresh = 50;

    public static double lossCompensator = 1.075, lowPassGain = 0.95;

    private double lastFilteredValue = 0;
    private double returnFilteredValue(double vRaw) {
        return lowPassGain * vRaw + (1-lowPassGain) * lastFilteredValue;
    }

    public static double vConstant = 12;
    private double vt = 0;

    public double getVt() {
        return vt;
    }

    public void runShooterSus() {
        encoderVelocity = shooter.getVelocity();

        rpm = ((encoderVelocity * 60) / 28) * velMult;
        readRPM = rpm;

        vt = vConstant / voltageSensor.getVoltage();

        if (rpm < kinematicRPMGoal - 50) {
            shooter2.setPower(1 * vt);
            shooter.setPower(1 * vt);
        } else {
            shooter2.setPower(kinematicRPMGoal * kF * vt);
            shooter.setPower(kinematicRPMGoal * kF * vt);
        }

        if (Double.isNaN(cachedMotorPower) || Math.abs(newPower - cachedMotorPower) > motorPowerCacheTolerance) {
            shooter.setPower(newPower);
            shooter2.setPower(newPower);
            cachedMotorPower = newPower;
        }

        setHoodAngle(producedHoodAngle);
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
        cachedMotorPower = p;
    }
    private double analogVoltageToDegrees(double voltage) {
        return voltage * (360/3.3);
    }
    public void stopShooter() {
        shooter.setPower(0);
        shooter2.setPower(0);
        cachedMotorPower = 0;
    }
    private double a = 0, b = 0, c = 0, n = 0, t_u = 0, t_g = 0,
            tof = 0, vX = 0, vY = 0, v = 0, m = 0;
    private double kinematicRPMGoal = 0, producedHoodAngle = 50;
    public static double w = 1.2;

    public double shooterVKinematic() {
        return v;
    }

    public double deltaServoAngle = 0;

    public double getDeltaServoAngle() {
        return deltaServoAngle;
    }

    public static double getLinkageHoodServoPos(double angleDeg) {
        return (0.00120419*(angleDeg*angleDeg*angleDeg*angleDeg)-0.266152*(angleDeg*angleDeg*angleDeg)+21.81051*(angleDeg*angleDeg)-790.9965*(angleDeg)+10871.9613)/270;
    }

    private double c1 = 2 * Math.PI * 0.036, c2 = 2 * Math.PI * 0.014 * 84/53;
    public static double entryAngleDeg = 45;
    private double entryAngleRad = Math.toRadians(entryAngleDeg);

    private double produceCounterRollerAdjustedVelocity(double v) {
        return (2 * v) / (c1 + c2);
    }

    public double getProducedHoodAngle() {
        return producedHoodAngle;
    }

    public static double startHoodDivisor = 4, endHoodDivisor = 9, endHoodMeters = 3;

    private double produceEntryAngle(double meters) {
        return meters <= 1 ? Math.PI/startHoodDivisor : (-(Math.PI/endHoodDivisor)/endHoodMeters) * (meters+1) + (Math.PI/4);
    }

    private double calculatedEntryAngle = 0.5235;

    public void updateFancyKinematics(double distMeters) {
        calculatedEntryAngle = Math.toRadians(getHoodAngle(distMeters));
        a = (-Math.tan(calculatedEntryAngle)*distMeters + w) / (distMeters * distMeters);
        b = -Math.tan(calculatedEntryAngle) - (2*a*distMeters);
        n = -b/(2*a);
        m = (a * (n*n)) + (b * (n)) + w;

        t_u = Math.sqrt((2*m) / 9.8);
        t_g = Math.sqrt((2*(m-w)) / 9.8);
        tof = t_u + t_g;

        vX = distMeters/tof;
        vY = (m - 0.5*(-9.8)*(t_u*t_u))/t_u;

        v = Math.sqrt((vX*vX) + (vY*vY));

        kinematicRPMGoal = produceCounterRollerAdjustedVelocity(v) * 60 * lossCompensator;
        producedHoodAngle = Math.toDegrees(calculatedEntryAngle);
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

    private int lastEncoderPos = 0;
    private long lastEncoderTimeNs = 0;

    public double getBulkVelocity() {
        int currentPos = shooter.getCurrentPosition(); // bulk cached
        long currentTimeNs = System.nanoTime();

        double velocity = (currentPos - lastEncoderPos) /
                ((currentTimeNs - lastEncoderTimeNs) / 1e9);

        lastEncoderPos = currentPos;
        lastEncoderTimeNs = currentTimeNs;
        return velocity;
    }
}
