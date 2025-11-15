package org.firstinspires.ftc.teamcode.systems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class Drivebase {

    private DcMotor fL, fR, bL, bR;

    private double xIn, yIn, zIn, normalizer;

    public static DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

    private boolean tele = false;

    public Drivebase(HardwareMap hardwareMap, boolean tele) {
        fL = hardwareMap.dcMotor.get("fL");
        bR = hardwareMap.dcMotor.get("bR");
        bL = hardwareMap.dcMotor.get("bL");
        fR = hardwareMap.dcMotor.get("fR");

        fL.setZeroPowerBehavior(zeroPowerBehavior);
        bL.setZeroPowerBehavior(zeroPowerBehavior);
        fR.setZeroPowerBehavior(zeroPowerBehavior);
        bR.setZeroPowerBehavior(zeroPowerBehavior);

        fL.setDirection(DcMotorSimple.Direction.REVERSE);
        bL.setDirection(DcMotorSimple.Direction.REVERSE);

        this.tele = tele;
    }

    public void takeTeleInput(double gpy, double gpx, double gpt) {
        xIn = gpy;
        yIn = gpx;
        zIn= gpt;
        normalizer = Math.max(Math.abs(xIn) + Math.abs(yIn) + Math.abs(zIn), 1);
    }

    public void update() {
        if (tele) {
            fL.setPower((xIn + yIn + zIn)/normalizer);
            bL.setPower((xIn - yIn + zIn)/normalizer);
            fR.setPower((xIn - yIn - zIn)/normalizer);
            bR.setPower((xIn + yIn - zIn)/normalizer);
        }
    }

    public void updateFC(double headingRad) {
        double rotX = xIn * Math.cos(headingRad) - yIn * Math.sin(headingRad);
        double rotY = xIn * Math.sin(headingRad) + yIn * Math.cos(headingRad);
        normalizer = Math.max(Math.abs(rotX) + Math.abs(rotY) + Math.abs(zIn), 1);
        fL.setPower((rotX + rotY + zIn)/normalizer);
        bL.setPower((rotX - rotY + zIn)/normalizer);
        fR.setPower((rotX - rotY - zIn)/normalizer);
        bR.setPower((rotX + rotY - zIn)/normalizer);
    }

}
