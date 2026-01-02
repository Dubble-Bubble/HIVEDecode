package org.firstinspires.ftc.teamcode.tests;

import static org.firstinspires.ftc.teamcode.tests.ShotAlgTest.c;

import android.util.Pair;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.systems.Shooter;
import org.firstinspires.ftc.teamcode.systems.Turret;

@TeleOp
@Config
public class SWMTest extends OpMode {

    private GoBildaPinpointDriver pinpoint;
    public static GoBildaPinpointDriver.EncoderDirection yDirection, xDirection;

    double yVel, xVel, turretAngle, tX, tY, encAngle, xPos, yPos, heading;
    boolean locked;

    private DcMotor transfer, intake;
    public static double intakePower = 0;
    private Turret turret;

    double limelightMountAngleDegrees = 10, limelightLensHeightInches = 13.4, goalHeightInches = 29.5, lastTurretTarget = 0;

    private Shooter shooter;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(xDirection, yDirection);

        pinpoint.setOffsets(-92.15, -111.625, DistanceUnit.MM);

        intake = hardwareMap.dcMotor.get("intake"); transfer = hardwareMap.dcMotor.get("transfer"); transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        shooter = new Shooter(hardwareMap, telemetry);

        turret = new Turret(hardwareMap, false);
        turret.setMode(Turret.Mode.odo);

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    double looptime = 0; boolean turretUpdateFlag = true;
    ElapsedTime looptimer = new ElapsedTime();

    @Override
    public void loop() {

        pinpoint.update();
        yVel = pinpoint.getVelX(DistanceUnit.METER);
        xVel = pinpoint.getVelY(DistanceUnit.METER);
        xPos = pinpoint.getPosX(DistanceUnit.INCH);
        yPos = pinpoint.getPosY(DistanceUnit.INCH);
        heading = pinpoint.getHeading(AngleUnit.DEGREES);

        double meters = turret.distanceToGoal(xPos, yPos) * 0.0254;

        encAngle = turret.getTurretAngle();
        encAngle = AngleUnit.normalizeDegrees(encAngle);
        encAngle = Math.toRadians(encAngle);

        double vyr = (yVel * Math.sin(Math.PI/2 - encAngle)) + (xVel*Math.sin(encAngle));
        double vxr = -(yVel * Math.sin(Math.PI/2 - encAngle)) + (xVel*Math.cos(encAngle));

        double vn = (((getRPMForShot(meters) + c)/60)*(0.036*2*Math.PI)) - vyr;

        double vt = Math.sqrt((vn*vn)+(vxr*vxr));
        telemetry.addData("vt", vt);

        double oo = Math.atan(-vxr/vn);
        telemetry.addData("offset angle", Math.toDegrees(oo));

        double trueShotVelocity = (vt/(Math.PI*2*0.036))*60;
        telemetry.addData("compensated velocity", trueShotVelocity);

        telemetry.addData("yVel", yVel);
        telemetry.addData("xVel", xVel);

        turret.setOffset(Math.toDegrees(oo));
        turret.setPose(new Pair<>(xPos, yPos), heading);
        turret.update();

        shooter.setTargetRPM(trueShotVelocity);
        shooter.setHoodAngle(getHoodAngle(meters));
        telemetry.addData("meters", meters);
        telemetry.addData("turret angle", encAngle);

        intake.setPower(intakePower);
        transfer.setPower(intakePower);

        shooter.runShooter();
        looptime = looptimer.milliseconds();
        telemetry.addData("loop time (ms)", looptime);
        telemetry.addData("loop time (hz)", (1000/looptime));
        telemetry.update();
        looptimer.reset();
    }

    private double analogVoltageToDegrees(double voltage) {
        return voltage * (360/3.3);
    }

    public double getRPMForShot(double meters) {
//        return (211.43 * meters) + 1177;
        return (227.87*meters) + 1382.7;
    }

    public double getHoodAngle(double meters) {
//        return (-8.8 * meters) + 76.16;
        return (-4.8701*meters) + 59.754;
    }
}
