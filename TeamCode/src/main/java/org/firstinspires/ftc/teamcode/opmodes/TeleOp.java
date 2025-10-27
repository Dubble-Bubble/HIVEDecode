package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.systems.Drivebase;
import org.firstinspires.ftc.teamcode.systems.Intake;
import org.firstinspires.ftc.teamcode.systems.Shooter;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "RedTeleop")
@Config
public class TeleOp extends OpMode {

    private Drivebase drivebase;
    private Intake intake;
    private DcMotor shooter;

    GamepadEx controller;

    private boolean isPressed = false;

    @Override
    public void init() {
        drivebase = new Drivebase(hardwareMap, true);
        intake = new Intake(hardwareMap);
        shooter = hardwareMap.dcMotor.get("shooter");
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        controller = new GamepadEx(gamepad1);
    }

    @Override
    public void loop() {
        drivebase.takeTeleInput(controller.getLeftY(), controller.getLeftX(), controller.getRightX());
        intake.setShootingMode(controller.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        intake.setActive(controller.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);

        if (gamepad1.dpad_up && !isPressed) {
            if (shooter.getPower() > 0.5) {
                shooter.setPower(0);
            } else {
                shooter.setPower(1);
            }
        } isPressed = gamepad1.dpad_up;

        intake.update();
        drivebase.update();
    }
}
