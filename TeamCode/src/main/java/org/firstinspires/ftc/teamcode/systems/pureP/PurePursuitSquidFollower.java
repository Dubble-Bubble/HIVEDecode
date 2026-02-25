package org.firstinspires.ftc.teamcode.systems.pureP;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDController;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDDrive;

@Config
public class PurePursuitSquidFollower {

    private SquIDController xC, yC, hC, fxC, fyC, fhC;
    public static double xp = 0.2, yp = -0.3, hp = 0.2, fxp = 0.05, fyp = -0.15, fhp = 0.4;
    private PurePursuitSquidDrive drive; private GoBildaPinpointDriver localizer;

    PurePursuitPath path; PurePursuitPath.PurePursuitOutput purePursuitOutput;

    public static double translationalTolerance = 2, headingTolerance = 2, fineHeadingThresh = 15, timeout = 1;

    public PurePursuitSquidFollower(PurePursuitSquidDrive drive, GoBildaPinpointDriver localizer) {
        this.drive = drive;
        this.localizer = localizer;
        xC = new SquIDController(xp);
        yC = new SquIDController(yp);
        hC = new SquIDController(hp);

        fxC = new SquIDController(fxp);
        fyC = new SquIDController(fyp);
        fhC = new SquIDController(fhp);
    }

    private Pose2D targetPose = new Pose2D(0,0,0), currentPose = new Pose2D(0,0,0), error = new Pose2D(0, 0, 0);
    private double x, y, z;

    public void setTargetPose(Pose2D targetPose) {
        this.targetPose = targetPose;
    }

    public void setCurrentPath(PurePursuitPath path) {
        this.path = path;
    }

    private double posX = 0, posY = 0, heading = 0;

    public void read() {
        localizer.update();
        currentPose = new Pose2D(localizer.getPosX(DistanceUnit.INCH), localizer.getPosY(DistanceUnit.INCH), localizer.getHeading(AngleUnit.RADIANS));
        purePursuitOutput = path.update(currentPose.x, currentPose.y);
        targetPose = new Pose2D(purePursuitOutput.targetX, purePursuitOutput.targetY, purePursuitOutput.targetHeading);
        error = subtract(targetPose, currentPose);
        error.set(new Pose2D(error.x, error.y, findHeadingError(currentPose.h, targetPose.h)));
    }

    ElapsedTime timeoutCheck = new ElapsedTime();

    public void update() {
        xC.setP(xp);
        yC.setP(yp);
        hC.setP(hp);

        fhC.setP(fhp);
        fyC.setP(fyp);
        fxC.setP(fxp);

        if (purePursuitOutput.atPathEnd) {
            x = fxC.calculate(currentPose.x, targetPose.x);
            y = fyC.calculate(currentPose.y, targetPose.y);
        } else {
            x = xC.calculate(currentPose.x, targetPose.x);
            y = yC.calculate(currentPose.y, targetPose.y);
        }

        if (error.h <= Math.toRadians(fineHeadingThresh)) {
            z = fhC.calculate(error.h);
        } else {
            z = hC.calculate(error.h);
        }

        if (!isFinished()) {
            drive.update(currentPose.h, x, y, z);
            timeoutCheck.reset();
        } else if (isFinished() && timeoutCheck.seconds() < timeout) {
            drive.update(currentPose.h, x, y, z);
        } else if (isFinished() && timeoutCheck.seconds() > timeout) {
            drive.stop();
        }
    }

    public double   findHeadingError(double cH, double tH) {
        return AngleUnit.normalizeRadians(tH-cH);
    }

    public boolean isFinished() {
       return Math.abs(error.x) <= translationalTolerance && Math.abs(error.y) <= translationalTolerance && Math.abs(error.h) <= Math.toRadians(headingTolerance);
    }

    public Pose2D subtract(Pose2D pose1, Pose2D pose2) {
        return new Pose2D(
                pose1.x - pose2.x,
                pose1.y - pose2.y,
                pose1.h - pose2.h
        );
    }

    public Pose2D getCurrentPose() {
        return currentPose;
    }

    public Pose2D getTargetPose() {
        return targetPose;
    }

    public void setCurrentPose(double x, double y, double headingRad) {
        currentPose = new Pose2D(x, y, headingRad);
        localizer.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, headingRad));
    }

    public Pose getPose() {
        return new Pose(currentPose.x, currentPose.y, currentPose.h);
    }

    public Pose2D getPinpointPose() {
        return new Pose2D(localizer.getPosX(DistanceUnit.INCH), localizer.getPosY(DistanceUnit.INCH), localizer.getHeading(AngleUnit.DEGREES));
    }
}
