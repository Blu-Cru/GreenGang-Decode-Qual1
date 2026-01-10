package org.firstinspires.ftc.teamcode.greengang.opmodes.test.tuner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive.DrivePID;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;

@Config
@TeleOp
public class DrivePIDTuner extends GreenLinearOpMode {
    public static double target = 0;

    private DrivePID drivePID;

    @Override
    public void initialize() {
        addDrivetrain();
        drivePID = new DrivePID();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void periodic() {
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;

        double turn = drivePID.getRotatePower(drivetrain.heading, Math.toRadians(target));

        if (Math.abs(turn) < 0.02) {
            drivePID.reset();
        }

        drivetrain.drive(drive, strafe, turn, false);

        telemetry.addData("Target (deh)", target);
        telemetry.addData("Heading (deg)", Math.toDegrees(drivetrain.heading));
        telemetry.addData("Turn", turn);
        telemetry.update();
    }

    @Override
    public void telemetry(Telemetry tele) {}
}
