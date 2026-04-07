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
import org.firstinspires.ftc.teamcode.systems.squid.SquIDFollower;

@Config
public class PurePursuitSquidFollower {

    private SquIDController xC, yC, hC, fxC, fyC, fhC;
    public static double xp = 0.15, yp = 0.25, hp = 0.65, fxp = 0.1, fyp = 0.15, fhp = 0.4;
    private SquIDDrive drive; private GoBildaPinpointDriver localizer;

    PurePursuitPath path; PurePursuitPath.PurePursuitOutput purePursuitOutput;

    public static double translationalTolerance = 2, headingTolerance = 2, fineHeadingThresh = 15, timeout = 1;
    private double getFineHeadingThreshRadians = Math.toRadians(fineHeadingThresh);

    public PurePursuitSquidFollower(SquIDDrive drive, GoBildaPinpointDriver localizer) {
        this.drive = drive;
        this.localizer = localizer;
        xC = new SquIDController(xp);
        yC = new SquIDController(yp);
        hC = new SquIDController(hp);

        fxC = new SquIDController(fxp);
        fyC = new SquIDController(fyp);
        fhC = new SquIDController(fhp);
    }

    private SquIDFollower.AdamPose targetPose = new SquIDFollower.AdamPose(0,0,0); SquIDFollower.AdamPose currentPose = new SquIDFollower.AdamPose(0,0,0),
            error = new SquIDFollower.AdamPose(    0, 0, 0);
    private double x, y, z;

    public void setTargetPose(SquIDFollower.AdamPose targetPose) {
        this.targetPose = targetPose;
    }

    public void setCurrentPath(PurePursuitPath path) {
        this.path = path;
    }

    private double posX = 0, posY = 0, heading = 0;

    public void read() {
        if (path == null) return;
        localizer.update();
        currentPose.set(localizer.getPosX(DistanceUnit.INCH), localizer.getPosY(DistanceUnit.INCH), localizer.getHeading(AngleUnit.RADIANS));
        purePursuitOutput = path.update(currentPose.getX(), currentPose.getY());
        targetPose.set(purePursuitOutput.targetX, purePursuitOutput.targetY, purePursuitOutput.targetHeading);
        error.set(targetPose.getX() - currentPose.getX(), targetPose.getY() - currentPose.getY(),
                findHeadingError(currentPose.getZ(), targetPose.getZ()));
    }

    public double getPurePursuitSuggestedHeading() {
        if (path == null) {
            return 676767;
        } else {
            return purePursuitOutput.targetHeading;
        }
    }

    public boolean atPathEnd() {
        if (path == null) {
            return false;
        } else {
            return purePursuitOutput.atPathEnd;
        }
    }

    ElapsedTime timeoutCheck = new ElapsedTime();

    public static boolean tuning = false;

    public void update() {

        if (purePursuitOutput == null) return;

        if (tuning) {
            xC.setP(xp);
            yC.setP(yp);
            hC.setP(hp);

            fhC.setP(fhp);
            fyC.setP(fyp);
            fxC.setP(fxp);
        }

        if (purePursuitOutput.atPathEnd) {
            x = fxC.calculate(currentPose.getX(), targetPose.getX());
            y = fyC.calculate(currentPose.getY(), targetPose.getY());
        } else {
            x = xC.calculate(currentPose.getX(), targetPose.getX());
            y = yC.calculate(currentPose.getY(), targetPose.getY());
        }

        if (Math.abs(error.getZ()) <= Math.toRadians(fineHeadingThresh)) {
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

    public double   findHeadingError(double cH, double tH) {
        return AngleUnit.normalizeRadians(tH-cH);
    }

    public boolean isFinished() {
       return Math.abs(error.getX()) <= translationalTolerance && Math.abs(error.getY()) <= translationalTolerance && Math.abs(error.getZ()) <= Math.toRadians(headingTolerance);
    }

    public Pose2D subtract(Pose2D pose1, Pose2D pose2) {
        return new Pose2D(
                pose1.x - pose2.x,
                pose1.y - pose2.y,
                pose1.h - pose2.h
        );
    }

    public SquIDFollower.AdamPose getCurrentPose() {
        return currentPose;
    }

    public SquIDFollower.AdamPose getTargetPose() {
        return targetPose;
    }

    public void setCurrentPose(double x, double y, double headingRad) {
        currentPose.set(x, y, headingRad);
        localizer.setPosition(new org.firstinspires.ftc.robotcore.external.navigation.Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, headingRad));
    }

    public Pose getPose() {
        return new Pose(currentPose.getX(), currentPose.getY(), currentPose.getZ());
    }

    public Pose2D getPinpointPose() {
        return new Pose2D(localizer.getPosX(DistanceUnit.INCH), localizer.getPosY(DistanceUnit.INCH), localizer.getHeading(AngleUnit.DEGREES));
    }
}
