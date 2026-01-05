package org.firstinspires.ftc.teamcode.commonA.drivetrainA;

import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive.PinpointLocalizer;
import org.firstinspires.ftc.teamcode.greengang.common.util.GreenSubsystem;
import org.firstinspires.ftc.teamcode.greengang.common.util.Globals;

public class Drivetrain implements GreenSubsystem, Subsystem {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public PinpointLocalizer pinpoint;

    public Pose2d pose;
    public double heading;

    public Drivetrain(HardwareMap hardwareMap) {
        pinpoint = hardwareMap.get(PinpointLocalizer.class, "pinpoint");

        frontLeft = hardwareMap.get(DcMotorEx.class, Globals.frontLeft);
        backLeft = hardwareMap.get(DcMotorEx.class, Globals.backLeft);
        frontRight = hardwareMap.get(DcMotorEx.class, Globals.frontRight);
        backRight = hardwareMap.get(DcMotorEx.class, Globals.backRight);

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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

    @Override
    public void init() {}

    @Override
    public void update() {
        pinpoint.update();
        pose = pinpoint.getPose();
        heading = pose.heading.toDouble();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("x", pose.position.x);
        telemetry.addData("y", pose.position.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(heading));
    }
}