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
    public static double runVoltage = 12.7;

    private VoltageSensor voltageSensor;

    public SquIDDrive(HardwareMap hardwareMap, VoltageSensor voltageSensor) {
        fl = hardwareMap.dcMotor.get("fL");
        bl = hardwareMap.dcMotor.get("bL");
        fr = hardwareMap.dcMotor.get("fR");
        br = hardwareMap.dcMotor.get("bR");

        fl.setZeroPowerBehavior(zeroPowerBehavior);
        bl.setZeroPowerBehavior(zeroPowerBehavior);
        fr.setZeroPowerBehavior(zeroPowerBehavior);
        br.setZeroPowerBehavior(zeroPowerBehavior);

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        this.voltageSensor = voltageSensor;
    }

    public void update(double heading, double x, double y, double z) {
        d = Math.max(abs(x) + abs(y) + abs(z), 1);
        applyGlobalPowers(heading, x, y, z, d);
    }

    private double abs(double i) {
        return Math.abs(i);
    }

    public void applyGlobalPowers(double heading, double x, double y, double z, double d) {
        double comp = runVoltage/voltageSensor.getVoltage();

        double xr = x * Math.cos(heading) - y * Math.sin(heading);
        double yr = x * Math.sin(heading) + y * Math.cos(heading);

        d = Math.max(abs(x) + abs(y) + abs(z), 1);
        fl.setPower(((xr+yr-z)/d) * comp);
        bl.setPower(((xr-yr-z)/d) * comp);
        fr.setPower(((xr-yr+z)/d) * comp);
        br.setPower(((xr+yr+z)/d) * comp);
    }

}
