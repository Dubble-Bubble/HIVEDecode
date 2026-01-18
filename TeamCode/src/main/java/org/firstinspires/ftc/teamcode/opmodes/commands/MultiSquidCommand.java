package org.firstinspires.ftc.teamcode.opmodes.commands;

import android.util.Pair;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import java.util.ArrayList;

public class MultiSquidCommand extends SequentialCommandGroup {

    private ArrayList<SparkFunOTOS.Pose2D> poses;
}
