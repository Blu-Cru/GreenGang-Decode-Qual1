package org.firstinspires.ftc.teamcode.greengang.common.util;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AprilTagMap {
    public static double BLUE_CORNER_X = 0;
    public static double BLUE_CORNER_Y = 135;

    public static double RED_CORNER_X = 144;
    public static double RED_CORNER_Y = 144;

    public static double[] getDistanceXY(double x, double y) {
        double goalX, goalY;

        if (Globals.alliance == Alliance.BLUE) {
            goalX = BLUE_CORNER_X;
            goalY = BLUE_CORNER_Y;
        } else {
            goalX = RED_CORNER_X;
            goalY = RED_CORNER_Y;
        }

        return new double[] {
                goalX - x,
                goalY - y
        };
    }
}
