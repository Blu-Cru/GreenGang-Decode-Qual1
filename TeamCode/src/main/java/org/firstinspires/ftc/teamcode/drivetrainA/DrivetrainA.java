package org.firstinspires.ftc.teamcode.drivetrainA;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DrivetrainA {
    public DcMotorEx FrontLeft = null;
    public DcMotorEx FrontRight = null;
    public DcMotorEx BackLeft = null;
    public DcMotorEx BackRight = null;
    public IMU imu = null;

    // Constructor
    public DrivetrainA(HardwareMap hardwareMap) {
        FrontLeft = hardwareMap.get(DcMotorEx.class, "leftFront");
        BackLeft = hardwareMap.get(DcMotorEx.class, "leftBack");
        FrontRight = hardwareMap.get(DcMotorEx.class, "rightFront");
        BackRight = hardwareMap.get(DcMotorEx.class, "rightBack");
        FrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
// Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
// Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
// Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
    }

    public void drive(double drive, double strafe, double turn, boolean yn) {
        if (yn) {
            drive = Math.pow(drive, 3);
            strafe = Math.pow(strafe, 3);
            turn = Math.pow(turn, 3);
        }

        double frontLeftPower = drive + strafe + turn;
        double frontRightPower = drive - strafe - turn;
        double backLeftPower = drive - strafe + turn;
        double backRightPower = drive + strafe - turn;
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }
        FrontLeft.setPower(frontLeftPower);
        FrontRight.setPower(frontRightPower);
        BackLeft.setPower(backLeftPower);
        BackRight.setPower(backRightPower);
    }


    public void RCdrive(Gamepad gamepad1, boolean yn) {
        double axial = -gamepad1.left_stick_y; // Note: pushing stick forward gives negative value
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;
        if (yn) {
            axial = Math.pow(axial, 3);
            lateral = Math.pow(lateral, 3);
            yaw = Math.pow(yaw, 3);
        }

        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }
        FrontLeft.setPower(frontLeftPower);
        FrontRight.setPower(frontRightPower);
        BackLeft.setPower(backLeftPower);
        BackRight.setPower(backRightPower);

    }
}
