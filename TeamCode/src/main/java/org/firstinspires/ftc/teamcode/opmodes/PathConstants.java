package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.systems.pureP.PurePursuitPath;

@Config
public class PathConstants {

    //Blue sidespike 1
    @Config
    public static class BlueSideSpikePath {
        public static double x0 = 32, x1 = 20, x2 = 24, 
                            y0 = 7.5, y1 = 10, y2 = 33;

    }

    //Far Return Path
    @Config
    public static class BlueReturnToFarPath {
        public static double x0 = BlueSideSpikePath.x2, x1 = 45,
                            y0 = BlueSideSpikePath.y2, y1 = 11;

    }

    //Far Return Path
    @Config
    public static class BlueCornerPath {
        public static double x0 = BlueSideSpikePath.x2, x1 = 9,
                y0 = BlueSideSpikePath.y2, y1 = 9;
    }

}
