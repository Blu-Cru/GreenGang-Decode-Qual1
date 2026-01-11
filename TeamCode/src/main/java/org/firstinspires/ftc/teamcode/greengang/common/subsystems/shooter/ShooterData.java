package org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter;

import com.acmerobotics.dashboard.config.Config;

@Config
public class ShooterData {

    public static final double A_VEL = 0.0193029;
    public static final double B_VEL = 8.58623;
    public static final double C_VEL = 1324.69907;

    //testing purposes
    public static boolean useLookupTable = true;

    public static double maxVelocity = 4000;

    //last values are just random for max distance possible to shoot and soemthing high enough so that the robot is basically at power 1
    private static final double[] DISTANCES = {
            14, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 170
    };

    private static final double[] VELOCITIES = {
            1600, 1575, 1575, 1575, 1600, 1625, 1650,
            1800, 1800, 1850, 1900, 1875, 2000, 2100,
            2250, 2300, 2450, 2505, 2545, 2610, 2750,
            2600, 2650, 2710, 2785, 2850, 2900, 4000,

    };

    public static double lookupTable(double distance){
        if (distance <= DISTANCES[0]) {
            return VELOCITIES[0];
        }

        for (int i = 0; i < DISTANCES.length - 1; i++) {
            double d0 = DISTANCES[i];
            double d1 = DISTANCES[i + 1];

            if (distance >= d0 && distance <= d1) {
                double v0 = VELOCITIES[i];
                double v1 = VELOCITIES[i + 1];

                double t = (distance - d0) / (d1 - d0);
                return v0 + t * (v1 - v0);
            }
        }

        return maxVelocity;
    }

    public static double velocityFromDistance(double d){
        if(useLookupTable) {
            return lookupTable(d);
        } else {
            return Math.min(maxVelocity, A_VEL * d * d + B_VEL * d + C_VEL);
        }
    }
}
