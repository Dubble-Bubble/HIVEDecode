package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.tests.ShotAlgTest.c;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.systems.Drivebase;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "SWMBlue (DO NOT USE UNLESS GARUNTEED WIN)")
@Config
public class BlueTeleSWM extends OpMode {

    private Drivebase drivebase; private Intake intake; private Shooter shooter; private Turret turret;
    public static double offset = 4, farOffset = 3, farTransferRate = 0.57, vcWeight = 0.37;

    GamepadEx controller;

    private boolean isDpadDownPressed = false, isBPressed = false, isAPressed = false, isDpadUpPressed = false, shootingMode = false, close = true, isYPressed = false;

    private GoBildaPinpointDriver pinpoint;
    public static GoBildaPinpointDriver.EncoderDirection yDirection, xDirection;

    public double xPos, yPos, meters, weight, driveMult = 1;

    Limelight3A limelight3A;

    boolean haveWeNanned = false;


    @Override
    public void init() {
        drivebase = new Drivebase(hardwareMap, true);
        intake = new Intake(hardwareMap);

        controller = new GamepadEx(gamepad1);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(xDirection, yDirection);

        pinpoint.setOffsets(-92.15, -111.625, DistanceUnit.MM);

        pinpoint.setPosition(PurpleAutoLimelight.endpose);

        turret = new Turret(hardwareMap, false);

        limelight3A = hardwareMap.get(Limelight3A.class, "ll3a");
        limelight3A.setPollRateHz(250);
        limelight3A.start();

        shooter = new Shooter(hardwareMap, telemetry);
    }

    double looptime = 0, hoodAngle = 0; boolean turretUpdateFlag = true;
    double xVel = 0, yVel = 0;
    ElapsedTime looptimer = new ElapsedTime();

    @Override
    public void loop() {

        looptimer.reset();

        telemetry.addData("loop time (ms)", looptime);
        telemetry.addData("loop time (hz)", (1000/looptime));

        pinpoint.update();

        if (gamepad1.b && !isBPressed) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 8.795, 8, AngleUnit.RADIANS, Math.toRadians(180)));
        } isBPressed = gamepad1.b;

        if (gamepad1.a && !isAPressed) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 144-8.795, 8, AngleUnit.RADIANS, 0));
        } isAPressed = gamepad1.a;

        double heading = pinpoint.getHeading(AngleUnit.RADIANS); xPos = pinpoint.getPosX(DistanceUnit.INCH); yPos = pinpoint.getPosY(DistanceUnit.INCH);
        xVel = pinpoint.getVelX(DistanceUnit.INCH); yVel = pinpoint.getVelY(DistanceUnit.INCH);
        drivebase.takeTeleInput(controller.getLeftY() * driveMult, controller.getLeftX() * driveMult, controller.getRightX() * driveMult);

        turret.setPose(new Pair<>(xPos, yPos), Math.toDegrees(heading));

        if (gamepad1.right_bumper && !isDpadDownPressed) {
            shootingMode = !shootingMode;
            close = true;
        } else if(gamepad1.dpad_down && !isDpadDownPressed) {
            shootingMode = !shootingMode;
            close = false;
        } isDpadDownPressed = gamepad1.right_bumper || gamepad1.dpad_down;

        if (gamepad1.left_bumper) {
            intake.setFlap(Intake.transferPosition);
            intake.setTransfer(true);
            intake.setActive(true);
            driveMult = 0.4;
        } else {
            intake.setFlap(Intake.lockedPosition);
            intake.setTransfer(false);
            intake.setActive(controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
            intake.setReverse(controller.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
            driveMult = 1;
        }

        if (shootingMode) {
            turret.setMode(Turret.Mode.odo);
            meters = turret.distanceToGoal(xPos, yPos) * 0.0254;
            System.out.println("Meters: ");
            System.out.println(meters);

            hoodAngle = shooter.getHoodAngle(meters);
            if (!haveWeNanned && Double.isNaN(hoodAngle)) {
                telemetry.addLine("hood is the NAN");
                haveWeNanned = true;
            }
            telemetry.addData("hoodAngle", hoodAngle);
            System.out.println("HoodAngle: ");
            System.out.println(hoodAngle);

            shooter.updateFancyKinematics(meters, Math.toRadians(hoodAngle));

            if (!Double.isNaN(shooter.getTof())) {
                weight = shooter.getTof() + (0.3);
            } else {
                weight = 0.3;
            }
            System.out.println("Weight: ");
            System.out.println(weight);
            if (!haveWeNanned && Double.isNaN(weight)) {
                telemetry.addLine("weight is the NAN");
                haveWeNanned = true;
            }

            turret.setAimPointOffset((-xVel*weight), (-yVel*weight));

            if (close) {
                intake.setTransferPower(1);
                intake.setIntakePower(1);

                double vyr = ((yVel * 0.025) * Math.sin(Math.PI/2 - turret.getOdoTarget())) + ((xVel * 0.025)*Math.sin(turret.getOdoTarget()));
                double vxr = -((yVel * 0.025) * Math.cos(Math.PI/2 - turret.getOdoTarget())) + ((xVel * 0.025)*Math.cos(turret.getOdoTarget()));

                double vn = (shooter.shooterVKinematic()) + (vyr * vcWeight);

                double vt = Math.sqrt((vn*vn)+(vxr*vxr));

                shooter.setTargetRPM(shooter.vMSToRPM(vt)+c);

                shooter.setHoodAngle(hoodAngle);
                Shooter.w = 1.2;
                telemetry.addData("target rpm", shooter.getKinematicRPMGoal() + c);
                telemetry.addData("tof estimate", shooter.getTof());
                turret.setOffset(-2);
                shooter.runShooterSus();
            } else {
                intake.setTransferPower(0.7);
                intake.setIntakePower(0.7);
                shooter.setTargetRPM(4200);
                shooter.setHoodAngle(47);
                turret.setOffset(-3.5);
                telemetry.addData("target rpm", 4120);
                shooter.runShooter();
            }

            telemetry.addData("meters", meters);


        } else {
            turret.setMode(Turret.Mode.fixed);
            turret.setTargetDegrees(0);
            intake.setIntakePower(1);
            turret.setOffset(-2);
            shooter.stopShooter();
        }

        telemetry.addData("is shootingModeOn", shootingMode);
        telemetry.addData("is close", close);
        telemetry.addData("pose x", xPos);
        telemetry.addData("pose y", yPos);
        telemetry.addData("heading", Math.toDegrees(heading));


        if (gamepad1.y && !isYPressed && !limelight3A.getLatestResult().getFiducialResults().isEmpty()) {
            Pose3D pose3 = limelight3A.getLatestResult().getBotpose();
            double rawHeadingRead = pose3.getOrientation().getYaw(AngleUnit.DEGREES);
            if (Math.signum(rawHeadingRead) < 0) {
                rawHeadingRead += 360;
            }
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 72+(pose3.getPosition().y*39.37), 72-(pose3.getPosition().x*39.37), AngleUnit.DEGREES,
                    rawHeadingRead-90));
        } isYPressed = gamepad1.y;

        intake.update();
        drivebase.update();
        turret.update();
        if (!haveWeNanned) {
            telemetry.update();
        }

        looptime = looptimer.milliseconds();
    }
}
