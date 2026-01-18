package org.firstinspires.ftc.teamcode.systems.squid;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDDrive;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDFollower;

import java.util.List;

@Config
@TeleOp
public class SquIDTuner extends OpMode {

    SquIDDrive drive;

    SquIDFollower follower;

    public static double tx, ty, th;

    private GoBildaPinpointDriver pinpoint;
    public static GoBildaPinpointDriver.EncoderDirection yDirection, xDirection;

    List<LynxModule> allHubs;

    @Override
    public void init() {

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        pinpoint.setOffsets(-92.15, -111.625, DistanceUnit.MM);

        pinpoint.resetPosAndIMU();

        drive = new SquIDDrive(hardwareMap, hardwareMap.voltageSensor.iterator().next());
        follower = new SquIDFollower(drive, pinpoint);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }
    }

    double looptime = 0; boolean turretUpdateFlag = true;
    ElapsedTime looptimer = new ElapsedTime();
    SparkFunOTOS.Pose2D cur;

    @Override
    public void loop() {

        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }

        looptimer.reset();

        telemetry.addData("loop time (ms)", looptime);
        telemetry.addData("loop time (hz)", (1000/looptime));

        follower.read();

        follower.setTargetPose(new SparkFunOTOS.Pose2D(tx, ty, Math.toRadians(th)));

        cur = follower.getCurrentPose();

        telemetry.addData("Current X", cur.x);
        telemetry.addData("Current Y", cur.y);
        telemetry.addData("Current H", cur.h);

        telemetry.addLine();

        telemetry.addData("isFinished?", follower.isFinished());
        telemetry.update();

        follower.update();

        looptime = looptimer.milliseconds();
    }
}
