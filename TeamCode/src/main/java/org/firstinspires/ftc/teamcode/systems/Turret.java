package org.firstinspires.ftc.teamcode.systems;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.ArrayList;
import java.util.Collections;

@Config
public class Turret {

    private Servo s1, s2, s3;
    private AnalogInput encoder;
    private Limelight3A ll3a;

    Mode mode = Mode.fixed;

    public static double p = 0.035, maxDelta;

    private Pair<Double, Double> redGoal = new Pair<>(144.0, 144.0), blueGoal = new Pair<>(0.0, 144.0);
    private Pair<Double, Double> pose;
    private double headingDeg = 0, targetDeg = 0, odoTarget = 0, turretAngle = 0, error = 0, trueTarget = 0, ltd = 0;

    public enum Mode {
        fixed,
        odo,
        limelight
    }

    private double xOffset = 0, yOffset = 0;

    public void setAimPointOffset(double xOffset, double yOffset) {
        this.xOffset = xOffset; this.yOffset = yOffset;
    }

    private boolean red = false;

    public void setTargetDegrees(double targetDegrees) {
        targetDeg = targetDegrees;
    }

    public Turret(HardwareMap hardwareMap, boolean red) {
        s1 = hardwareMap.servo.get("cts");
        s2 = hardwareMap.servo.get("mts");
        s3 = hardwareMap.servo.get("fts");

        encoder = hardwareMap.analogInput.get("encoder");
        this.red = red;
    }

    public void setPose(Pair<Double, Double> pose, double headingDeg) {
        this.pose = pose;
        if (Math.signum(headingDeg) < 0) {
            this.headingDeg = 360+headingDeg;
        } else {
            this.headingDeg = headingDeg;
        }
    }

    ElapsedTime runtime = new ElapsedTime();

    private double offset = 0;

    public void setOffset(double offset) {
        this.offset = offset;
    }

    public void update() {
        runtime.reset();
        turretAngle = analogVoltageToDegrees(encoder.getVoltage());

        switch (mode) {
            case odo:
                if (red) {
                    odoTarget = Math.atan(
                            ((redGoal.second+yOffset) - pose.second)/((redGoal.first+xOffset) - pose.first)
                    );
                } else {
                    odoTarget = Math.atan2(
                            ((blueGoal.second+yOffset) - pose.second),((blueGoal.first+xOffset) - pose.first)
                    );
                }

                targetDeg = Math.toDegrees(odoTarget)-headingDeg;
                break;
            case fixed:
                break;
            case limelight:
                error = 0-(ll3a.getLatestResult().getTx());
                break;
        }


        setAngle(targetDeg + offset);
    }

    public double getTargetAngle() {
        return targetDeg;
    }

    public double getError() {
        return error;
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getCalculatedAngleByPosition() {
        return Math.toDegrees(odoTarget);
    }

    public void limelightReset() {

    }

    private double pos = 0;

    private void setAngle(double angle) {
        pos = interpolateAngle(angle);
        s1.setPosition(pos);
        s2.setPosition(pos);
        s3.setPosition(pos);
    }

    private double interpolateAngle(double angle) {
        angle = AngleUnit.normalizeDegrees(angle);
        if (angle > 165) {
            angle = 165;
        } else if (angle < -155) {
            angle = -155;
        }

        return (angle + 155) / 320;

    }

    private double analogVoltageToRadians(double voltage) {
        return voltage * ((2*Math.PI)/3.3);
    }

    private double analogVoltageToDegrees(double voltage) {
        return voltage * ((360)/3.3);
    }

    public void setMode(Mode mode) {
        this.mode = mode;
    }

    public double distanceToGoal(double xPosition, double yPosition) {
        if (red) {
            return Math.hypot(((redGoal.first+xOffset)-xPosition), ((redGoal.second+yOffset)-yPosition));

        } else {
            return Math.hypot(((blueGoal.first+xOffset)-xPosition), ((blueGoal.second+yOffset)-yPosition));
        }
    }

    public double getFitP(double error) {
        return  Math.max(0.556 + (-0.086*Math.log(error)), 0.03);
    }
}
