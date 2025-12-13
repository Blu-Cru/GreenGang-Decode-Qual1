package org.firstinspires.ftc.teamcode.greengang.opmodes.test.tuner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;

@Config
@TeleOp(group = "test")
public class DrivePIDTuner extends GreenLinearOpMode {
    public static double targetHeading = 0;

    private boolean lock = false;

    @Override
    public void initialize() {
        addDrivetrain();
    }

    @Override
    public void onStart() {
        targetHeading = drivetrain.heading;
        drivetrain.pid.setTargetHeading(targetHeading);
    }

    @Override
    public void periodic() {
        drivetrain.update();

        if (gamepad1.a) {
            lock = !lock;
        }

        if (lock) {
            drivetrain.pid.setTargetHeading(targetHeading);
            double rot = drivetrain.pid.getRotatePower(drivetrain.heading);
            drivetrain.fieldCentricDrive(0, 0, rot);
        } else {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double rot = -gamepad1.right_stick_x;
            drivetrain.fieldCentricDrive(x, y, rot);
            drivetrain.pid.setTargetHeading(drivetrain.heading);
        }
    }

    @Override
    public void telemetry(Telemetry tele) {
        tele.addData("Lock", lock);
        tele.addData("Target Heading", targetHeading);
        tele.addData("Current Heading", drivetrain.heading);
    }
}
