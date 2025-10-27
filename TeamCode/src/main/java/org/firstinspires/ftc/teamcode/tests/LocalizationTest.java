package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.systems.Localizer;

@TeleOp
public class LocalizationTest extends OpMode {

    Localizer localizer;

    @Override
    public void init() {
        localizer = new Localizer(hardwareMap, true);
        localizer.setPose(new Pose2D(DistanceUnit.INCH, -(72-7.5), 24-7.5, AngleUnit.DEGREES, 0));
    }

    @Override
    public void loop() {
        localizer.update();
        Pose2D pose2D = localizer.getTruePose();
        telemetry.addData("x", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("y", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("z", pose2D.getHeading(AngleUnit.DEGREES));

    }
}
