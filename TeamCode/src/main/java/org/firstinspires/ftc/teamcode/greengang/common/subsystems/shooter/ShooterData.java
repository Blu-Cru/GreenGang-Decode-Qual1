package org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter;

public class ShooterData {
    public static final double A_HOOD = 0;
    public static final double B_HOOD = 0;
    public static final double C_HOOD = 0;

    public static final double A_VEL = 0;
    public static final double B_VEL = 0;
    public static final double C_VEL = 0;

    public static double hoodFromDistance(double d){
        return A_HOOD * d * d + B_HOOD * d + C_HOOD;
    }

    public static double velocityFromDistance(double d){
        return A_VEL * d * d + B_VEL * d + C_VEL;
    }
}
