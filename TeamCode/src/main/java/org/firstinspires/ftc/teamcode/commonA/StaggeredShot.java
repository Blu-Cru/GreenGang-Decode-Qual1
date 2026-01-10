package org.firstinspires.ftc.teamcode.commonA;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commonA.intakeA.IntakeA;
import org.firstinspires.ftc.teamcode.commonA.outtakeA.OuttakeA;
import org.firstinspires.ftc.teamcode.greengang.opmodes.tele.TeleQual;

public class StaggeredShot {
    private enum State { READY, INTAKE_RUNNING, RECOVERING, NA}
    private static State currentState = State.READY;
    private static ElapsedTime timer = new ElapsedTime();

    // Tuning Constants
    public static double DROP_THRESHOLD = 100.0; // RPM drop
    public static double RECOVERY_DELAY = 0.2;   // 200ms delay

    public static void update(boolean buttonPressed, double targetRPM, double currentRPM, IntakeA intake, OuttakeA outtake, boolean on) {
        if (!on) {
            currentState = State.NA;
        }
        else if (!buttonPressed) {
            currentState = State.READY;
            intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN);

        }



        switch (currentState) {
            case READY:
                outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                if (Math.abs(currentRPM - targetRPM) < TeleQual.ALLOWED_VELO_ERR) {
                    outtake.setOuttakeState(OuttakeA.OuttakeState.SPIN_UP);
                    currentState = State.INTAKE_RUNNING;
                }
                break;

            case INTAKE_RUNNING:

                //if rpm drops too low beneath the target then it must have just shot and can now wait
                if (currentRPM < (targetRPM - DROP_THRESHOLD)) {
                    outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                    timer.reset(); // Start 0.3s cooldown
                    currentState = State.RECOVERING;
                }
                break;

            case RECOVERING:

                if (timer.seconds() > RECOVERY_DELAY && Math.abs(currentRPM - targetRPM) < TeleQual.ALLOWED_VELO_ERR) {
                    currentState = State.READY;
                }
                break;
            case NA:
                break;
        }
    }
}
