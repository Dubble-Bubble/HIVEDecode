package org.firstinspires.ftc.teamcode.opmodes.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.systems.CameraSubsystem;
import org.firstinspires.ftc.teamcode.systems.squid.SquIDFollower;

public class FindAndTargetArtifactClump extends CommandBase {

    CameraSubsystem subsystem;

    double executionTime = 600;

    boolean first = true, red = true;

    ElapsedTime elapsedTime = new ElapsedTime();

    SquIDFollower follower;

    Telemetry telemetry;

    public FindAndTargetArtifactClump(CameraSubsystem camera, double executionTimeMs, SquIDFollower follower, boolean red, Telemetry telemetry) {
        subsystem = camera;
        this.follower = follower;
        this.executionTime = executionTimeMs;
        this.red = red;
        this.telemetry = telemetry;
    }


    @Override
    public boolean isFinished() {
        if (first) {
            elapsedTime.reset();
            first = false;
            return false;
        }
        if (elapsedTime.milliseconds() > executionTime){
            SquIDFollower.headingTolerance = 9;
            SquIDFollower.translationalTolerance = 12;
            follower.setTargetPose(subsystem.getClumpLocation(follower.getCurrentPose(), red));
            telemetry.addLine("Detected");
            return true;
        } else {
            telemetry.addLine("detecting");
            return false;
        }
    }

}
