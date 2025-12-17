package org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

@Config
public class DrivePID {

    public static double kPHeading = 0, kIHeading = 0, kDHeading = 0;

    private PIDController headingController;
    private double targetHeading;

    public DrivePID() {
        headingController = new PIDController(kPHeading, kIHeading, kDHeading);
        targetHeading = 0;
    }

    public void setTargetHeading(double target) {
        targetHeading = angleWrap(target);
        headingController.setSetPoint(0);
    }
    
    public void reset() {
        headingController.reset();
    }

    public double getRotatePower(double currentHeading) {
        headingController.setPID(kPHeading, kIHeading, kDHeading);

        double error = angleWrap(targetHeading - angleWrap(currentHeading));
        double output = headingController.calculate(error, 0);

        return Range.clip(output, -1, 1);
    }

    // https://www.ctrlaltftc.com/practical-examples/controlling-heading
    public double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
