package org.firstinspires.ftc.teamcode.greengang.opmodes.tele;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commonA.drivetrainA.DrivetrainA;
import org.firstinspires.ftc.teamcode.commonA.intakeA.IntakeA;
import org.firstinspires.ftc.teamcode.commonA.outtakeA.OuttakeA;

@TeleOp(name = "RCSMTeleOp")
public class RCSMTeleOp extends CommandOpMode {

    // 1. Define Subsystems
    private IntakeA intake;
    private OuttakeA outtake; // Replaces raw flywheel, kicker, hood variables
    private DrivetrainA drivetrain;

    // 2. Define States
    public enum RobotState {
        START,
        INTAKE_IN,
        INTAKE_IN_SHOOT,
        INTAKE_OUT,
        OUTTAKE,
        SHOOT
    }

    RobotState currentState = RobotState.START;

    // Variables
    private ElapsedTime runtime = new ElapsedTime();
    // Hood position is still stored here because it's manually adjustable
    private double pos_hood=0;

    @Override
    public void initialize() {
        // --- HARDWARE INIT ---
        intake = new IntakeA(hardwareMap);
        drivetrain = new DrivetrainA(hardwareMap);


        // Initialize Outtake Subsystem
        outtake = new OuttakeA(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    public void run() {
        super.run(); // Run the scheduler

        // --------------------------------------------------------
        // 1. INPUT HANDLING (Determine the State)
        // --------------------------------------------------------

        // Priority 1: Both Triggers -> INTAKE_IN_SHOOT
        if ((gamepad1.left_trigger > 0.1 && gamepad1.right_trigger > 0.1) || (gamepad2.left_trigger > 0.1 && gamepad2.right_trigger > 0.1)) {
            currentState = RobotState.INTAKE_IN_SHOOT;
        }
        // Priority 2: Right Trigger + Bumper -> SHOOT
        else if ((gamepad1.right_trigger > 0.1 && gamepad1.right_bumper) || (gamepad2.right_trigger > 0.1 && gamepad2.right_bumper)){
            currentState = RobotState.SHOOT;
        }
        // Priority 3: Right Trigger Only -> OUTTAKE
        else if (gamepad1.right_trigger > 0.1 || gamepad2.right_trigger > 0.1) {
            currentState = RobotState.OUTTAKE;
        }
        // Priority 4: Left Trigger Only -> INTAKE_IN
        else if (gamepad1.left_trigger > 0.1 || gamepad2.left_trigger > 0.1) {
            currentState = RobotState.INTAKE_IN;
        }
        // Priority 5: Square Button -> INTAKE_OUT
        else if (gamepad1.square || gamepad2.square) {
            currentState = RobotState.INTAKE_OUT;
        }
        // Default: Idle
        else {
            currentState = RobotState.START;
        }

        // --------------------------------------------------------
        // 2. STATE MACHINE (Execute Logic based on State)
        // --------------------------------------------------------

        switch (currentState) {
            case START:
                intake.setIntakeState(IntakeA.IntakeState.IDLE);
                outtake.setOuttakeState(OuttakeA.OuttakeState.OFF); // New clean command
                break;

            case INTAKE_IN:
                intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN);
                outtake.setOuttakeState(OuttakeA.OuttakeState.OFF);
                break;

            case INTAKE_IN_SHOOT:
                intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN_SHOOT);
                outtake.setOuttakeState(OuttakeA.OuttakeState.SPIN_UP); // Spin up flywheel
                break;

            case INTAKE_OUT:
                intake.setIntakeState(IntakeA.IntakeState.INTAKE_OUT);
                outtake.setOuttakeState(OuttakeA.OuttakeState.REVERSE); // Flywheel reverse for jams
                break;

            case OUTTAKE:
                intake.setIntakeState(IntakeA.IntakeState.IDLE);
                outtake.setOuttakeState(OuttakeA.OuttakeState.SPIN_UP); // Flywheel only
                break;

            case SHOOT:
                intake.setIntakeState(IntakeA.IntakeState.IDLE);
                outtake.setOuttakeState(OuttakeA.OuttakeState.FIRE); // Fire kicker
                break;
        }

        // --------------------------------------------------------
        // 3. MANUAL CONTROLS & TELEMETRY
        // --------------------------------------------------------

        drivetrain.RCdrive(gamepad1, gamepad1.circle);

        // Hood Control (Still handled directly in TeleOp because it's a dynamic setting)
        if (gamepad1.dpad_down || gamepad2.dpad_down) pos_hood -= 0.001;
        else if (gamepad1.dpad_up || gamepad2.dpad_up) pos_hood += 0.001;

        if (gamepad1.cross || gamepad2.cross) pos_hood = 0.05;
        if (gamepad1.triangle || gamepad2.cross) pos_hood = 0.12;
        if (gamepad1.dpad_left || gamepad2.dpad_left) {
            double v = 1950.0;
            outtake.setTarget_Vel(v);
        }
        else if (gamepad1.dpad_right || gamepad2.dpad_right) {
            double v = 2350.0;
            outtake.setTarget_Vel(v);
        }

        outtake.setHoodPosition(pos_hood); // Call the hood setter in the subsystem

        // Telemetry
        telemetry.addData("Robot State", currentState);
        telemetry.addData("Outtake State", outtake.getOuttakeState()); // Check flywheel state
        telemetry.addData("Intake State", intake.getIntakeState());
        telemetry.addData("hood_pos", outtake.getHoodPos());
        telemetry.addData("fly_pow", outtake.getFlyVel());
        telemetry.update();
    }
}