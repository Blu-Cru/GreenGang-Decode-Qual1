package org.firstinspires.ftc.teamcode.commonA.outtakeA;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class OuttakeA extends SubsystemBase{

    private final DcMotorEx m_flywheel1, m_flywheel2;
    private final Servo m_kicker;
    private final Servo m_hood;
    private final Servo m_stop;

    public enum OuttakeState {
        OFF,
        REVERSE,
        FIRE,
        REV,
        SPIN_UP
    }

    private OuttakeState m_currentState = OuttakeState.OFF;

    //private final double SHOOT_POWER = 1.0;
    private final double REVERSE_POWER = -1.0;
    private final double KICKER_RETRACTED_POS = 0.8;
    private final double KICKER_FIRE_POS = 0.3;
    private double target_Vel = 2200;
    public static double kP = 0.0025, kI = 0.005, kD = 0.0001, ff = 0.00037;
    public static double target, velocity = 0;
    PIDController controller;

    public OuttakeA(final HardwareMap hardwareMap) {
        m_flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        m_flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        m_kicker = hardwareMap.get(Servo.class, "kicker");
        m_hood = hardwareMap.get(Servo.class, "hood");
        m_stop = hardwareMap.get(Servo.class, "hardstop");
        /*m_flywheel1.setDirection(DcMotorSimple.Direction.REVERSE);
        m_flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);*/
        controller = new PIDController(kP, kI, kD);

        setOuttakeState(OuttakeState.OFF);
        m_hood.setPosition(0);


        m_flywheel1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m_flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        m_flywheel2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        m_flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }



    public void setOuttakeState(OuttakeState newState) {
        m_currentState = newState;
    }

    public void setHoodPosition(double position) {
        m_hood.setPosition(position);
    }

    public OuttakeState getOuttakeState() {
        return m_currentState;
    }
    public double getHoodPos() {
        return m_hood.getPosition();
    }
    public void setTarget_Vel(double target_temp) {
        target_Vel=target_temp;
    }
    public void updatePid(){
        controller.setPID(kP, kI, kD);
    }
    public double getFlyVel() {
        return m_flywheel1.getVelocity();
    }

    public void setMotorPower(double power){
        m_flywheel1.setPower(power);
        m_flywheel2.setPower(power);
    }

    public void runFlywheelPID(){
        double power = 0;

        target = target_Vel;
        power = controller.calculate(getFlyVel(), target) + ff  * target;

        setMotorPower(power);
    }

    @Override
    public void periodic() {
        switch (m_currentState) {
            case OFF:
                setMotorPower(0);

                m_kicker.setPosition(KICKER_RETRACTED_POS);
                m_stop.setPosition(1);
                break;
            case REVERSE:
                setMotorPower(REVERSE_POWER);

                m_kicker.setPosition(KICKER_RETRACTED_POS);
                m_stop.setPosition(0);
                break;
            case FIRE:
                runFlywheelPID();

                m_kicker.setPosition(KICKER_FIRE_POS);
                m_stop.setPosition(0);
                break;
            case REV:
                runFlywheelPID();

                m_kicker.setPosition(KICKER_RETRACTED_POS);
                m_stop.setPosition(1);
                break;
<<<<<<< Updated upstream
<<<<<<< Updated upstream
=======
=======
>>>>>>> Stashed changes

>>>>>>> Stashed changes
            case SPIN_UP:
                runFlywheelPID();

                m_kicker.setPosition(KICKER_RETRACTED_POS);
                m_stop.setPosition(0);
                break;
        }
    }
}