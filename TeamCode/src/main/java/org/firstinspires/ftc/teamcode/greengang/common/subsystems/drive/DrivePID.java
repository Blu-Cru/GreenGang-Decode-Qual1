package org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.util.Range;

@Config
public class DrivePID {

    public static double kPHeading = 2.05, kIHeading = 0, kDHeading = 1.88;

    public PIDController headingController;

    public DrivePID() {
        headingController = new PIDController(kPHeading, kIHeading, kDHeading);
    }
    
    public void reset() {
        headingController.reset();
    }

    public double getRotatePower(double currentHeading, double targetHeading) {
        headingController.setPID(kPHeading, kIHeading, kDHeading);

        double error = angleWrap(angleWrap(targetHeading) - currentHeading);
        double output = headingController.calculate(0, error);

        return Range.clip(output, -1, 1);
    }

    // https://www.ctrlaltftc.com/practical-examples/controlling-heading
    public double angleWrap(double radians) {
        while (radians > Math.PI) radians -= 2 * Math.PI;
        while (radians < -Math.PI) radians += 2 * Math.PI;
        return radians;
    }
}
