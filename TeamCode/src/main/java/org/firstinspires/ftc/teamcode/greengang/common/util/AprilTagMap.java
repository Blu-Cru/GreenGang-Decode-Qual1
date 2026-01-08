package org.firstinspires.ftc.teamcode.greengang.common.util;

public class AprilTagMap {
    //blue
    public static double TARGET_BLUE_X = 0;
    public static double TARGET_BLUE_Y = 134;

    public static double TARGET_RED_X = 144;
    public static double TARGET_RED_Y = 144;

    public static double[] getDistanceXY(double x, double y) {
        double dx, dy;

        if (Globals.alliance == Alliance.BLUE) {
            dx = TARGET_BLUE_X - x;
            dy = TARGET_BLUE_Y - y;
        } else {
            dx = TARGET_RED_X - x;
            dy = TARGET_RED_Y - y;
        }

        return new double[]{dx, dy};
    }
}
