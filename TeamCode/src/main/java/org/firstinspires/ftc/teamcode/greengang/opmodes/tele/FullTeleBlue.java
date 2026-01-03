package org.firstinspires.ftc.teamcode.greengang.opmodes.tele;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.commonA.drivetrainA.DrivetrainA;
import org.firstinspires.ftc.teamcode.commonA.intakeA.IntakeA;
import org.firstinspires.ftc.teamcode.commonA.outtakeA.OuttakeA;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "BlueTele_Command_Sniper", group = "TeleOp")
public class FullTeleBlue extends CommandOpMode {

    // --- SUBSYSTEMS ---
    private DrivetrainA drivetrain;
    private IntakeA intake;
    private OuttakeA outtake;
    private Follower follower;
    private Limelight3A limelight;
    private VoltageSensor batterySensor;

    // --- AUTO-AIM TARGET ---
    public static double TARGET_X = 11;
    public static double TARGET_Y = 132;

    // --- TUNABLES ---
    public static double SNAP_P = 2.05;
    public static double SNAP_D = 1.88;
    public static double COEFF_A = 0.045, COEFF_B = 12.5, COEFF_C = 1600.0;
    public static double HOOD_BASE = 0.1, HOOD_SLOPE = 0.00001;
    public static double VELO_GAIN = 1.15, VOLT_NOMINAL = 13.0;

    // --- STATE MACHINE ---
    public enum RobotState {
        START, INTAKE_IN, INTAKE_IN_SHOOT, INTAKE_OUT, OUTTAKE, SHOOT
    }
    private RobotState currentState = RobotState.START;

    private double lastError = 0;
    private double pos_hood = 0;
    private double manualRPM = 2100;

    @Override
    public void initialize() {
        // Hardware Mapping
        intake = new IntakeA(hardwareMap);
        outtake = new OuttakeA(hardwareMap);
        intake.setIntakeState(IntakeA.IntakeState.IDLE);
        intake.periodic();
        outtake.setOuttakeState(OuttakeA.OuttakeState.OFF);
        outtake.periodic();


        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(45, 120, Math.toRadians(180)));

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.start();

        batterySensor = hardwareMap.voltageSensor.iterator().next();
        drivetrain = new DrivetrainA(hardwareMap);
        telemetry.addData("Status", "Command System Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        // CRITICAL: Tells the FTCLib scheduler to run active commands
        super.run();

        // 1. Update Localization
        follower.update();
        Pose currPose = follower.getPose();

        // 2. Ballistics Solver (Calculate auto values every loop)
        double curX = currPose.getX();
        double curY = currPose.getY();
        double curHeading = currPose.getHeading();
        double rx = follower.getVelocity().getXComponent();
        double ry = follower.getVelocity().getYComponent();

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
        double autoAimRPM = (baseRPM - ((rx * (dx / vDist)) + (ry * (dy / vDist)) * VELO_GAIN)) * (VOLT_NOMINAL / batterySensor.getVoltage());
        double autoAimHood = Range.clip(HOOD_BASE - (vDist * HOOD_SLOPE), 0.1, 0.6);

        // 3. State Machine Logic (Inputs)
        if ((gamepad1.left_trigger > 0.1 && gamepad1.right_trigger > 0.1) || (gamepad2.left_trigger > 0.1 && gamepad2.right_trigger > 0.1)) {
            currentState = RobotState.INTAKE_IN_SHOOT;
        } else if ((gamepad1.right_trigger > 0.1 && gamepad1.right_bumper) || (gamepad2.right_trigger > 0.1 && gamepad2.right_bumper)) {
            currentState = RobotState.SHOOT;
        } else if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
            currentState = RobotState.OUTTAKE;
        } else if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
            currentState = RobotState.INTAKE_IN;
        } else if (gamepad1.square || gamepad2.square) {
            currentState = RobotState.INTAKE_OUT;
        } else {
            currentState = RobotState.START;
        }

        // Execute State Logic
        switch (currentState) {
            case START:
                intake.setIntakeState(IntakeA.IntakeState.IDLE);
                outtake.setOuttakeState(OuttakeA.OuttakeState.OFF);
                break;
            case INTAKE_IN:
                intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN);
                outtake.setOuttakeState(OuttakeA.OuttakeState.OFF);
                break;
            case INTAKE_IN_SHOOT:
                intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN_SHOOT);
                outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                break;
            case INTAKE_OUT:
                intake.setIntakeState(IntakeA.IntakeState.INTAKE_OUT);
                outtake.setOuttakeState(OuttakeA.OuttakeState.REVERSE);
                break;
            case OUTTAKE:
                intake.setIntakeState(IntakeA.IntakeState.IDLE);
                outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                break;
            case SHOOT:
                intake.setIntakeState(IntakeA.IntakeState.IDLE);
                outtake.setOuttakeState(OuttakeA.OuttakeState.FIRE);
                break;
        }

        // 4. Drive Logic & Sniper Lock
        double drive = -gamepad1.left_stick_y;
        double strafe = -gamepad1.left_stick_x;
        double turn;

        if (gamepad1.right_trigger > 0.1) {
            // Lock Heading
            double targetHeading = Math.atan2(dy, dx);
            double error = targetHeading - curHeading;
            while (error > Math.PI) error -= 2 * Math.PI;
            while (error < -Math.PI) error += 2 * Math.PI;

            turn = (error * SNAP_P) + ((error - lastError) * SNAP_D);
            lastError = error;

            // Apply Ballistics automatically
            outtake.setTarget_Vel(autoAimRPM);
            outtake.setHoodPosition(autoAimHood);
        } else {
            // Manual Mode
            turn = -gamepad1.right_stick_x;
            lastError = 0;

            // Manual adjustments
            if (gamepad1.dpad_left || gamepad2.dpad_left) manualRPM = 1950.0;
            else if (gamepad1.dpad_right || gamepad2.dpad_right) manualRPM = 2350.0;

            if (gamepad1.dpad_down || gamepad2.dpad_down) pos_hood -= 0.001;
            else if (gamepad1.dpad_up || gamepad2.dpad_up) pos_hood += 0.001;

            if (gamepad1.cross || gamepad2.cross) pos_hood = 0.05;
            if (gamepad1.triangle || gamepad2.triangle) pos_hood = 0.12;

            outtake.setTarget_Vel(manualRPM);
            outtake.setHoodPosition(pos_hood);
        }

        drivetrain.drive(drive, -strafe, -turn, true);
        if (gamepad1.share && gamepad1.options) {
            follower.setPose(new Pose(125, 3, 0));
        }

        // 5. Telemetry
        telemetry.addData("State", currentState);
        telemetry.addData("Lock", gamepad1.right_trigger > 0.1 ? "ON" : "OFF");
        telemetry.addData("Dist", "%.2f", vDist);
        telemetry.addData("X/Y", "%.1f, %.1f", curX, curY);
        telemetry.update();
    }
}
