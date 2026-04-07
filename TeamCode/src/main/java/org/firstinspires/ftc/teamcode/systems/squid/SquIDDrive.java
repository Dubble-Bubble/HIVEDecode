package org.firstinspires.ftc.teamcode.systems.squid;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;

@Config
public class SquIDDrive {

    private DcMotor fl, fr, bl, br;

    private double d;

    public static DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;
    public static double runVoltage = 12;

    private VoltageSensor voltageSensor;

    public static double motorCacheTol = 0.01;
    private double cachedFl = Double.NaN;
    private double cachedFr = Double.NaN;
    private double cachedBl = Double.NaN;
    private double cachedBr = Double.NaN;

    private static final long vRefreshRate = 250;
    private double cachedVoltage = 12.0;
    private long lastVoltageReadMs = 0;

    public SquIDDrive(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
        fl = hardwareMap.dcMotor.get("fL");
        bl = hardwareMap.dcMotor.get("bL");
        fr = hardwareMap.dcMotor.get("fR");
        br = hardwareMap.dcMotor.get("bR");

        fl.setZeroPowerBehavior(zeroPowerBehavior);
        bl.setZeroPowerBehavior(zeroPowerBehavior);
        fr.setZeroPowerBehavior(zeroPowerBehavior);
        br.setZeroPowerBehavior(zeroPowerBehavior);

        fr.setDirection(DcMotorSimple.Direction.REVERSE);
        br.setDirection(DcMotorSimple.Direction.REVERSE);

        this.voltageSensor = voltageSensor;
    }

    public void update(double heading, double x, double y, double z) {
        d = Math.max(Math.abs(x) + Math.abs(y) + Math.abs(z), 1);
        applyGlobalPowers(heading, x, y, z, d);
    }

    public void applyGlobalPowers(double heading, double x, double y, double z, double d) {
        double comp = runVoltage / getVoltage();

        double xr = x * Math.cos(heading) + y * Math.sin(heading);
        double yr = -x * Math.sin(heading) + y * Math.cos(heading);

        writeMotor(fl, ((xr - yr - z) / d) * comp, cachedFl);
        cachedFl = ((xr - yr - z) / d) * comp;

        writeMotor(bl, ((xr + yr - z) / d) * comp, cachedBl);
        cachedBl = ((xr + yr - z) / d) * comp;

        writeMotor(fr, ((xr + yr + z) / d) * comp, cachedFr);
        cachedFr = ((xr + yr + z) / d) * comp;

        writeMotor(br, ((xr - yr + z) / d) * comp, cachedBr);
        cachedBr = ((xr - yr + z) / d) * comp;
    }

    public void stop() {
        fl.setPower(0); cachedFl = 0;
        bl.setPower(0); cachedBl = 0;
        fr.setPower(0); cachedFr = 0;
        br.setPower(0); cachedBr = 0;
    }

    private void writeMotor(DcMotor motor, double power, double lastPower) {
        if (Double.isNaN(lastPower) || Math.abs(power - lastPower) > motorCacheTol) {
            motor.setPower(power);
        }
    }

    private double getVoltage() {
        long now = System.currentTimeMillis();
        if (now - lastVoltageReadMs > vRefreshRate) {
            cachedVoltage = voltageSensor.getVoltage();
            lastVoltageReadMs = now;
        }
        return cachedVoltage;
    }
}