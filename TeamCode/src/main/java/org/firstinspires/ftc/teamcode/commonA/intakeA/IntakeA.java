package org.firstinspires.ftc.teamcode.commonA.intakeA;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeA extends SubsystemBase {
    private DcMotorEx intake = null;
    private CRServo lift = null;

    public enum IntakeState {
        IDLE,
        INTAKE_IN, // Forward power
        INTAKE_IN_SHOOT,
        INTAKE_OUT// Reverse power
    }
    private IntakeState intake_state = IntakeState.IDLE;
    private final double INTAKE_POWER = 1.0;
    private final double SPIT_POWER = -1.0; // Use negative for reverse direction
    public IntakeA(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");



    }


    public void setIntakeState(IntakeState newState) {
        intake_state = newState;
    }
    public IntakeState getIntakeState() {
        return intake_state;
    }
    @Override
    public void periodic() {
        switch (intake_state) {
            case IDLE:
                intake.setPower(0);


                break;
            case INTAKE_IN:
                intake.setPower(1);


                break;
            case INTAKE_OUT:
                intake.setPower(-1);


                break;

        }
    }

}

