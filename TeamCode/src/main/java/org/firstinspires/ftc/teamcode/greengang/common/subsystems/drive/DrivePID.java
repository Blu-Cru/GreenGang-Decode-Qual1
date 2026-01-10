package org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

@Config
public class DrivePID {
    public static double kPHeading = 2;
    public static double kIHeading = 0.25;
    public static double kDHeading = 0.14;

    private final PIDController controller;

    public DrivePID() {
        controller = new PIDController(kPHeading, kIHeading, kDHeading);
    }

    public void reset() {
        controller.reset();
    }

    public double getRotatePower(double currentHeading, double targetHeading) {
        controller.setPID(kPHeading, kIHeading, kDHeading);

        double error = angleWrap(targetHeading - currentHeading);
        double output = controller.calculate(error, 0);

        return Range.clip(output, -1.0, 1.0);
    }

    private double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}