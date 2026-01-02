package org.firstinspires.ftc.teamcode.tests;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.systems.Localizer;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;

@Config
@TeleOp
public class OdoTurretTest extends OpMode {

    Turret turret;
    Shooter shooter;
    private GoBildaPinpointDriver pinpoint;
    public static GoBildaPinpointDriver.EncoderDirection yDirection, xDirection;

    public double xPos, yPos, meters;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(xDirection, yDirection);

        pinpoint.setOffsets(-92.15, -111.625, DistanceUnit.MM);

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 144-112, 135.600, AngleUnit.DEGREES, 270));

        turret = new Turret(hardwareMap, false);
        shooter = new Shooter(hardwareMap, telemetry);
        turret.setMode(Turret.Mode.odo);
    }

    @Override
    public void loop() {
            pinpoint.update();

            xPos = pinpoint.getPosX(DistanceUnit.INCH); yPos = pinpoint.getPosY(DistanceUnit.INCH);
            turret.setPose(new Pair<>(xPos, yPos), pinpoint.getHeading(AngleUnit.DEGREES));

            meters = turret.distanceToGoal(xPos, yPos) * 0.0254;
            shooter.setTargetRPM(shooter.getRPMForShot(meters)+1500);
            shooter.setHoodAngle(shooter.getHoodAngle(meters));

            telemetry.addData("x: ", xPos);
            telemetry.addData("y: ", yPos);
            telemetry.addData("z: ", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("meter distance; ", meters);

            telemetry.addData("turret angle degrees", turret.getTurretAngle());
            telemetry.addData("turret target angle", turret.getTargetAngle());
            telemetry.addData("turret error", turret.getError());
            telemetry.addData("Calculated Angle Based on Odo", turret.getCalculatedAngleByPosition());

            telemetry.update();
            turret.update();
            shooter.runShooter();
    }
}
