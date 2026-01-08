package org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive;


import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.util.AprilTagMap;
import org.firstinspires.ftc.teamcode.greengang.common.util.GreenSubsystem;
import org.firstinspires.ftc.teamcode.greengang.common.util.Globals;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

public class Drivetrain implements GreenSubsystem, Subsystem {
    public Follower follower;
    public DrivePID drivePID;

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public Pose pose;
    public double heading;


    double targetHeading = 0;
    double error = 0;

    public Drivetrain(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotorEx.class, Globals.frontLeft);
        backLeft = hardwareMap.get(DcMotorEx.class, Globals.backLeft);
        frontRight = hardwareMap.get(DcMotorEx.class, Globals.frontRight);
        backRight = hardwareMap.get(DcMotorEx.class, Globals.backRight);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        drivePID = new DrivePID();

        follower = Constants.createFollower(hardwareMap);
    }

    public void drive(double drive, double strafe, double turn, boolean cubic) {
        if (cubic) {
            drive = Math.pow(drive, 3);
            strafe = Math.pow(strafe, 3);
            turn = Math.pow(turn, 3);
        }

        double fl = drive + strafe + turn;
        double fr = drive - strafe - turn;
        double bl = drive - strafe + turn;
        double br = drive + strafe - turn;

        double max = Math.max(Math.max(Math.abs(fl), Math.abs(fr)), Math.max(Math.abs(bl), Math.abs(br)));

        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }

        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    public void fieldCentricDrive(Gamepad gamepad1) {
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        y = Math.pow(y, 3);
        x = Math.pow(x, 3);
        rx = Math.pow(rx, 3);

        double rotX = x * Math.cos(-heading) - y * Math.sin(-heading);
        double rotY = x * Math.sin(-heading) + y * Math.cos(-heading);

        rotX *= 1.1;

        double denom = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1.0);

        frontLeft.setPower((rotY + rotX + rx) / denom);
        backLeft.setPower((rotY - rotX + rx) / denom);
        frontRight.setPower((rotY - rotX - rx) / denom);
        backRight.setPower((rotY + rotX - rx) / denom);
    }

    public void teleOpDrive(Gamepad gamepad1){
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn = 0;

        if (Globals.autoAimEnabled) {
            double[] distances = AprilTagMap.getDistanceXY(pose.getX(), pose.getY());
            
            double dx = distances[0];
            double dy = distances[1];
            
            targetHeading = Math.atan2(dy, dx);

            turn = drivePID.getRotatePower(heading, targetHeading);
        } else {
            turn = -gamepad1.right_stick_x;

            drivePID.reset();
        }

        drive(drive, -strafe, -turn, true);
    }

    @Override
    public void init() {
        follower.setStartingPose(Globals.startPose);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        pose = follower.getPose();
    }

    @Override
    public void update() {
        follower.update();

        pose = follower.getPose();
        heading = pose.getHeading();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("x", pose.getX());
        telemetry.addData("y", pose.getY());
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
        telemetry.addData("Drivetrain Target", targetHeading);
        telemetry.addData("Drivetrain Error", error);
    }
}