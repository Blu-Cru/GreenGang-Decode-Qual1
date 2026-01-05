package org.firstinspires.ftc.teamcode.commonA.outtakeA;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.util.GreenSubsystem;

@Config
public class ShooterA implements GreenSubsystem, Subsystem {
    public final double KICKER_RETRACTED_POS = 0.8;
    public final double KICKER_FIRE_POS = 0.3;
    public double targetVelocity = 0;
    public static double DEFAULT_VEL = 2200;
    public static double kP = 0.0025, kI = 0.005, kD = 0.0001, ff = 0.00037;
    public static double target = 0;

    DcMotorEx flywheel1, flywheel2;
    Servo kicker;
    Servo hood;
    Servo hardstop;
    PIDController controller;

    public enum State {
        IDLE,
        REVERSE,
        FIRE,
        REV,
        SPIN_UP
    }

    public State state = State.IDLE;

    public void setState(State newState) {
        state = newState;
    }

    public void setHoodPosition(double position) {
        hood.setPosition(position);
    }

    public State getState() {
        return state;
    }
    public double getHoodPos() {
        return hood.getPosition();
    }
    public void setTargetVelocity(double t) {
        targetVelocity = t;
    }
    public void updatePid(){
        controller.setPID(kP, kI, kD);
    }
    public double getFlywheelVelocity() {
        return flywheel1.getVelocity();
    }
    public void stop(){ state = State.IDLE; }

    public void setMotorPower(double power){
        flywheel1.setPower(power);
        flywheel2.setPower(power);
    }

    public void runFlywheelPID(){
        double power = 0;

        target = targetVelocity;
        power = controller.calculate(getFlywheelVelocity(), target) + ff  * target;

        setMotorPower(power);
    }

    public ShooterA(HardwareMap hardwareMap) {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        kicker = hardwareMap.get(Servo.class, "kicker");
        hood = hardwareMap.get(Servo.class, "hood");
        hardstop = hardwareMap.get(Servo.class, "hardstop");

        controller = new PIDController(kP, kI, kD);

        setState(State.IDLE);
    }

    @Override
    public void init() {
        hood.setPosition(0);

        flywheel1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        flywheel2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        switch (state) {
            case IDLE:
                setMotorPower(0);

                kicker.setPosition(KICKER_RETRACTED_POS);
                hardstop.setPosition(1);
                break;
            case REVERSE:
                setMotorPower(-1);

                kicker.setPosition(KICKER_RETRACTED_POS);
                hardstop.setPosition(0);
                break;
            case FIRE:
                kicker.setPosition(KICKER_FIRE_POS);
                hardstop.setPosition(0);
                break;
            case REV:
                kicker.setPosition(KICKER_RETRACTED_POS);
                hardstop.setPosition(1);
                break;
            case SPIN_UP:

                kicker.setPosition(KICKER_RETRACTED_POS);
                hardstop.setPosition(0);
                break;
        }

        runFlywheelPID();
    }

    @Override
    public void telemetry(Telemetry telemetry) {
        telemetry.addData("State", state);
    }
}