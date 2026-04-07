package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.tests.ShotAlgTest.c;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.opmodes.v1stuff.PurpleAutoLimelight;
import org.firstinspires.ftc.teamcode.systems.Drivebase;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.PinpointConstants;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Blue Teleop")
@Config
public class BlueTele extends OpMode {

    private Drivebase drivebase; private Intake intake; private Shooter shooter; private Turret turret;
    public static double offset = 4, farOffset = -4.5, farTransferRate = 0.57, vcWeight = 0, farRPM = 4500, farHood = 45;

    GamepadEx controller;

    private boolean isDpadDownPressed = false, isBPressed = false, isAPressed = false, isDpadUpPressed = false, shootingMode = false, close = true, turretIncrPressed = false;

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

        PinpointConstants.initializePinpoint(pinpoint);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.RADIANS, 0));

        turret = new Turret(hardwareMap, false);

//        limelight3A = hardwareMap.get(Limelight3A.class, "ll3a");
//        limelight3A.setPollRateHz(250);
//        limelight3A.start();

        shooter = new Shooter(hardwareMap, telemetry);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        drivebase.takeTeleInput(controller.getLeftY() * driveMult, controller.getLeftX() * driveMult, controller.getRightX() * driveMult);

        turret.setPose(new Pair<>(xPos, yPos), Math.toDegrees(heading));

        if (gamepad1.right_bumper && !isDpadDownPressed) {
            shootingMode = !shootingMode;
            close = true;
        }  isDpadDownPressed = gamepad1.right_bumper;

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

            shooter.updateFancyKinematics(meters);

            if (close) {
                intake.setTransferPower(1);
                intake.setIntakePower(1);

                telemetry.addData("target rpm", shooter.getKinematicRPMGoal());
                telemetry.addData("suggested hood angle", shooter.getProducedHoodAngle());
                telemetry.addData("suggested servo angle delta", shooter.getDeltaServoAngle());
                telemetry.addData("suggested hood servo position", shooter.getLinkageHoodServoPos(shooter.getProducedHoodAngle()));
                telemetry.addData("rpm estimate", shooter.getRpm());
                telemetry.addData("voltage comp term", shooter.getVt());
                shooter.runShooterSus();
            }

            telemetry.addData("meters", meters);

        } else {
            turret.setMode(Turret.Mode.fixed);
            turret.setTargetDegrees(0);
            intake.setIntakePower(1);
            shooter.stopShooter();
        }

        telemetry.addData("is shootingModeOn", shootingMode);
        telemetry.addData("is close", close);
        telemetry.addData("pose x", xPos);
        telemetry.addData("pose y", yPos);
        telemetry.addData("heading", Math.toDegrees(heading));

        if (gamepad2.left_bumper && !turretIncrPressed) {
            turret.incrementOffset(true);
        } else if (gamepad2.right_bumper && !turretIncrPressed) {
            turret.incrementOffset(false);
        } turretIncrPressed = gamepad2.left_bumper || gamepad2.right_bumper;

        intake.update();
        drivebase.update();
        turret.update();

        looptime = looptimer.milliseconds();
    }
}
