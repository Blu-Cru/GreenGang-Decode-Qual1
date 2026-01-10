package org.firstinspires.ftc.teamcode.greengang.opmodes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.commonA.drivetrainA.DrivetrainA;
import org.firstinspires.ftc.teamcode.commonA.outtakeA.OuttakeA;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Sniper_Manual_Tuner", group = "TeleOp")
public class AutoAimTester extends OpMode {

    private DrivetrainA drivetrain;
    private OuttakeA outtake;
    private Follower follower;
    private Limelight3A limelight;
    private VoltageSensor batterySensor;

    public static double TARGET_X = 10;
    public static double TARGET_Y = 132;

    public static double COEFF_A = 0.045, COEFF_B = 12.5, COEFF_C = 1350.0;
    public static double HOOD_BASE = 0.22, HOOD_SLOPE = 0.0012;
    public static double VELO_GAIN = 1.15, VOLT_NOMINAL = 13.0;

    // --- TUNABLE GAINS ---
    public static double SNAP_P = 2.05;
    public static double SNAP_D = 1.88;
    private double lastError = 0;

    // Button state tracking for tuning
    private boolean lastUp = false, lastDown = false, lastLeft = false, lastRight = false;

    @Override
    public void init() {

        outtake = new OuttakeA(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        batterySensor = hardwareMap.voltageSensor.iterator().next();

        follower.setStartingPose(new Pose(45, 120, Math.toRadians(180)));
        drivetrain = new DrivetrainA(hardwareMap);
        limelight.start();
    }

    @Override
    public void loop() {
        follower.update();



        // Grabbing raw coordinates directly
        double curX = follower.getPose().getX();
        double curY = follower.getPose().getY();
        double curHeading = follower.getPose().getHeading();
        double rx = follower.getVelocity().getXComponent();
        double ry = follower.getVelocity().getYComponent();

        // --- BALLISTICS ---
        double tof = 0.5, vDist = 0, baseRPM = 0;
        double vGoalX = TARGET_X, vGoalY = TARGET_Y;

        for (int i = 0; i < 3; i++) {
            vGoalX = TARGET_X - (rx * tof);
            vGoalY = TARGET_Y - (ry * tof);
            vDist = Math.hypot(vGoalX - curX, vGoalY - curY);
            baseRPM = (COEFF_A * Math.pow(vDist, 2)) + (COEFF_B * vDist) + COEFF_C;
            tof = vDist / Math.max(baseRPM * 0.32, 10.0);
        }

        double dx = vGoalX - curX;
        double dy = vGoalY - curY;
        double finalRPM = (baseRPM - ((rx * (dx / vDist)) + (ry * (dy / vDist)) * VELO_GAIN)) * (VOLT_NOMINAL / batterySensor.getVoltage());

        // --- DRIVE LOGIC ---
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn;

        if (gamepad1.right_trigger > 0.1) {
            double targetHeading = Math.atan2(dy, dx);
            double error = targetHeading - curHeading;

            // Angle Wrap
            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;

            // PD Calculation
            turn = (error * SNAP_P) + ((error - lastError) * SNAP_D);
            lastError = error;
        } else {
            turn = -gamepad1.right_stick_x;
            lastError = 0;
        }

        // --- THE BYPASS COMMAND ---
        drivetrain.drive(drive, -strafe, -turn, true);

        // --- OUTPUT ---
        outtake.setTarget_Vel(finalRPM);
        outtake.setHoodPosition(Range.clip(HOOD_BASE - (vDist * HOOD_SLOPE), 0.1, 0.6));


        telemetry.addData("Dist", vDist);
        telemetry.addData("Heading Error", Math.toDegrees(lastError));
        telemetry.update();
    }
}