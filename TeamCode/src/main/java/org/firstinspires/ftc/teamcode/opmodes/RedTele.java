package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.tests.ShotAlgTest.c;
import static org.firstinspires.ftc.teamcode.tests.ShotAlgTest.f;

import android.util.Pair;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.systems.Drivebase;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Localizer;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RedTeleop")
@Config
public class RedTele extends OpMode {

    private Drivebase drivebase; private Intake intake; private Shooter shooter; private Turret turret;

    GamepadEx controller;

    private boolean isDpadDownPressed = false, isBPressed = false, isAPressed = false, isDpadUpPressed = false, shootingMode = false, close = true, flapUp = false;

    private GoBildaPinpointDriver pinpoint;
    public static GoBildaPinpointDriver.EncoderDirection yDirection, xDirection;

    public double xPos, yPos, meters;


    @Override
    public void init() {
        drivebase = new Drivebase(hardwareMap, true);
        intake = new Intake(hardwareMap);

        controller = new GamepadEx(gamepad1);

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(xDirection, yDirection);

        pinpoint.setOffsets(-92.15, -111.625, DistanceUnit.MM);

        pinpoint.setPosition(RedAutoLimelight.endpose);

        turret = new Turret(hardwareMap, true);

        shooter = new Shooter(hardwareMap, telemetry);
    }

    double looptime = 0; boolean turretUpdateFlag = true;
    ElapsedTime looptimer = new ElapsedTime();

    @Override
    public void loop() {

        looptimer.reset();

        telemetry.addData("loop time (ms)", looptime);
        telemetry.addData("loop time (hz)", (1000/looptime));

        pinpoint.update();

        if (gamepad1.a && !isAPressed) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 8.795, 8, AngleUnit.RADIANS, Math.toRadians(180)));
        } isAPressed = gamepad1.a;

        if (gamepad1.b && !isBPressed) {
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 144-8.795, 8, AngleUnit.RADIANS, 0));
        } isBPressed = gamepad1.b;

        double heading = pinpoint.getHeading(AngleUnit.RADIANS); xPos = pinpoint.getPosX(DistanceUnit.INCH); yPos = pinpoint.getPosY(DistanceUnit.INCH);
        drivebase.takeTeleInput(controller.getLeftY(), controller.getLeftX(), controller.getRightX());

        turret.setPose(new Pair<>(xPos, yPos), Math.toDegrees(heading));

        if (gamepad1.right_bumper && !isDpadDownPressed) {
            shootingMode = !shootingMode;
            close = true;
        } else if(gamepad1.dpad_down && !isDpadDownPressed) {
            shootingMode = !shootingMode;
            close = false;
        } isDpadDownPressed = gamepad1.right_bumper || gamepad1.dpad_down;

        if (gamepad1.left_bumper) {
            intake.setFlap(Intake.flapUp);
            intake.setTransfer(true);
            intake.setActive(true);
        } else {
            intake.setFlap(Intake.flapDown);
            intake.setTransfer(false);
            intake.setActive(controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);
            intake.setReverse(controller.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        }

        if (shootingMode) {
            turret.setMode(Turret.Mode.odo);
            meters = turret.distanceToGoal(xPos, yPos) * 0.0254;
            if (close) {
                shooter.setTargetRPM(shooter.getRPMForShot(meters) + c);
                shooter.setHoodAngle(shooter.getHoodAngle(meters));
                telemetry.addData("target rpm", shooter.getRPMForShot(meters) + c);
            } else {
                shooter.setTargetRPM(shooter.getRPMForShot(meters) + f);
                shooter.setHoodAngle(Math.min(shooter.getHoodAngle(meters), 47));
                telemetry.addData("target rpm", shooter.getRPMForShot(meters) + f);
            }

            telemetry.addData("meters", meters);

            shooter.runShooter();
        } else {
            turret.setMode(Turret.Mode.fixed);
            turret.setTargetDegrees(0);
            shooter.stopShooter();
        }

        telemetry.addData("is shootingModeOn", shootingMode);
        telemetry.addData("is close", close);
        telemetry.addData("pose x", xPos);
        telemetry.addData("pose y", yPos);
        telemetry.addData("heading", Math.toDegrees(heading));

        intake.update();
        drivebase.update();
        turret.update();
        telemetry.update();

        looptime = looptimer.milliseconds();
    }
}
