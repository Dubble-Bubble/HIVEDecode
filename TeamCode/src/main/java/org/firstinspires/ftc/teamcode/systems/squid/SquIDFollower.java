package org.firstinspires.ftc.teamcode.systems.squid;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDDrive;

@Config
public class SquIDFollower {

    private SquIDController xC, yC, hC, fxC, fyC, fhC;
    public static double xp = 0.2, yp = 0.3, hp = 0.6, fxp = 0.05, fyp = 0.15, fhp = 0.4;

    public static double fineTranslationalThresh = 10, fineHeadingThresh = 10, timeout = 1;
    public static double getFineHeadingThreshRadians = Math.toRadians(fineHeadingThresh);
    private SquIDDrive drive; private GoBildaPinpointDriver localizer;

    public static double translationalTolerance = 2, headingTolerance = 2;

    public SquIDFollower(SquIDDrive drive, GoBildaPinpointDriver localizer) {
        this.drive = drive;
        this.localizer = localizer;
        xC = new SquIDController(xp);
        yC = new SquIDController(yp);
        hC = new SquIDController(hp);

        fxC = new SquIDController(fxp);
        fyC = new SquIDController(fyp);
        fhC = new SquIDController(fhp);
    }

    private Pose2D targetPose = new Pose2D(0,0,0); AdamPose currentPose = new AdamPose(0,0,0), error = new AdamPose(    0, 0, 0);
    private double x, y, z;

    public void setTargetPose(Pose2D targetPose) {
        this.targetPose = targetPose;
    }

    private double posX = 0, posY = 0, heading = 0;

    public void read() {
        localizer.update();
        currentPose.set(localizer.getPosX(DistanceUnit.INCH), localizer.getPosY(DistanceUnit.INCH), localizer.getHeading(AngleUnit.RADIANS));
        error.set(targetPose.x - currentPose.getX(), targetPose.y - currentPose.getY(),
                findHeadingError(currentPose.getZ(), targetPose.h));
    }

    ElapsedTime timeoutCheck = new ElapsedTime();

    public static boolean tuning = false;

    public void update() {

        if (tuning) {
            xC.setP(xp);
            yC.setP(yp);
            hC.setP(hp);

            fhC.setP(fhp);
            fyC.setP(fyp);
            fxC.setP(fxp);
        }

        if (totalTranslationError(error.x, error.y) <= fineTranslationalThresh) {
            x = fxC.calculate(currentPose.x, targetPose.x);
            y = fyC.calculate(currentPose.y, targetPose.y);
        } else {
            x = xC.calculate(currentPose.x, targetPose.x);
            y = yC.calculate(currentPose.y, targetPose.y);
        }

        if (Math.abs(error.getZ()) <= getFineHeadingThreshRadians) {
            z = fhC.calculate(error.getZ());
        } else {
            z = hC.calculate(error.getZ());
        }

        if (!isFinished()) {
            drive.update(currentPose.getZ(), x, y, z);
            timeoutCheck.reset();
        } else if (isFinished() && timeoutCheck.seconds() < timeout) {
            drive.update(currentPose.getZ(), x, y, z);
        } else if (isFinished() && timeoutCheck.seconds() > timeout) {
            drive.stop();
        }
    }

    public double totalTranslationError(double xE, double yE) {
        return Math.sqrt(Math.pow(xE, 2) + Math.pow(yE, 2));
    }

    public double   findHeadingError(double cH, double tH) {
        return AngleUnit.normalizeRadians(tH-cH);
    }

    public boolean isFinished() {
       return Math.abs(error.x) <= translationalTolerance &&
               Math.abs(error.y) <= translationalTolerance &&
               Math.abs(error.getZ()) <= Math.toRadians(headingTolerance);
    }

    public Pose2D subtract(Pose2D pose1, Pose2D pose2) {
        return new Pose2D(
                pose1.x - pose2.x,
                pose1.y - pose2.y,
                pose1.h - pose2.h
        );
    }

    public Pose2D getCurrentPose() {
        return new Pose2D(currentPose.getX(), currentPose.getY(), currentPose.getZ());
    }

    public Pose2D getTargetPose() {
        return targetPose;
    }

    public void setCurrentPose(double x, double y, double headingRad) {
        currentPose.set(x, y, headingRad);
        localizer.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, headingRad));
    }

    public Pose getPose() {
        return new Pose(currentPose.getX(), currentPose.getY(),
                currentPose.getZ());
    }

    public AdamPose getPinpointPose() {
        return currentPose;
    }

    public static class AdamPose {
        private double x = 0, y = 0, z = 0;

        public AdamPose(double x, double y, double z) {
            this.x = x; this.y = y; this.z = z;
        }

        public void setY(double y) {
            this.y = y;
        }

        public void setX(double x) {
            this.x = x;
        }

        public void setZ(double z) {
            this.z = z;
        }

        public double getY() {
            return y;
        }

        public double getX() {
            return x;
        }

        public double getZ() {
            return z;
        }

        public void set(double x, double y, double z) {
            this.x = x; this.y = y; this.z = z;
        }
    }
}
