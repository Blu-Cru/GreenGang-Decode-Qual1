package org.firstinspires.ftc.teamcode.greengang.opmodes.test.tuner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive.DrivePID;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;

@Config
@TeleOp
public class DrivePIDTuner extends GreenLinearOpMode {
    public static double targetHeading = 0;

    public DrivePID drivePID;

    public double turn = 0;

    @Override
    public void initialize() {
        addDrivetrain();

        drivePID = new DrivePID();
    }

    @Override
    public void periodic() {
        turn = drivePID.getRotatePower(drivetrain.heading, Math.toRadians(targetHeading));

        drivetrain.drive(0, 0, turn, true);
    }

    @Override
    public void telemetry(Telemetry tele) {
        tele.addData("Target", targetHeading);
        tele.addData("Heading", drivetrain.heading);
        tele.addData("Turn Power", turn);
    }
}
