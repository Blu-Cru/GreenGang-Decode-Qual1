package org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.greengang.common.util.GreenSubsystem;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;

public class Drivetrain implements GreenSubsystem, Subsystem {
    public final MecanumDrive drive;
    public final DrivePID pid;

    public double drivePower = 1.0;
    public double heading;
    public Pose2d pose;

    public double deadzone = 0.05;
    public boolean driverControlTurn = true;

    public Drivetrain(HardwareMap hardwareMap) {
        drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));
        pid = new DrivePID();
    }

    public void relocalizeFromLimelight(Pose3D limelightPose) {
        double x = limelightPose.getPosition().x;
        double y = limelightPose.getPosition().y;
        double h = limelightPose.getOrientation().getYaw(AngleUnit.RADIANS);

        relocalize(new Pose2d(x, y, h));
    }


    public void relocalize(Pose2d newPose) {
        drive.localizer.setPose(newPose);

        pose = newPose;
        heading = newPose.heading.toDouble();

        pid.setTargetHeading(heading);
    }

    public void teleOpDrive(Gamepad g1) {
        double y = -g1.left_stick_y;
        double x = g1.left_stick_x;
        double turnInput = -g1.right_stick_x;

        boolean turningRightStick = Math.abs(turnInput) > deadzone;

        if (g1.right_stick_button) {
            driverControlTurn = !driverControlTurn;
            g1.rumbleBlips(driverControlTurn ? 1 : 2);
        }

        if (driverControlTurn) {
            if (turningRightStick) {
                pid.setTargetHeading(heading);
                fieldCentricDrive(x, y, turnInput);
            } else {
                fieldCentricDrive(x, y, 0);
            }
        } else {
            double rot = pid.getRotatePower(heading);
            fieldCentricDrive(x, y, rot);
        }
    }

    public void fieldCentricDrive(double x, double y, double rot) {
        double rotX = x * Math.cos(heading) - y * Math.sin(heading);
        double rotY = x * Math.sin(heading) + y * Math.cos(heading);

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(rotX * drivePower, rotY * drivePower),
                rot * drivePower
        ));
    }

    public void idle() {
        drive.setDrivePowers(new PoseVelocity2d(new Vector2d(0, 0), 0));
    }

    @Override
    public void update() {
        drive.updatePoseEstimate();
        pose = drive.localizer.getPose();
        heading = pose.heading.toDouble();
    }


    public void telemetry(Telemetry tele) {
        tele.addData("x", pose.position.x);
        tele.addData("y", pose.position.y);
        tele.addData("heading (deg)", Math.toDegrees(heading));
        tele.addData("drive power", drivePower);
        tele.addData("PID enabled", !driverControlTurn);
    }

    @Override
    public void init() {}
}
