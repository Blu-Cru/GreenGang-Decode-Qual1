package org.firstinspires.ftc.teamcode.greengang.common.subsystems.intake;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.util.GreenSubsystem;

public class Intake implements GreenSubsystem, Subsystem {
    public enum State {
        IDLE,
        IN,
        SPIT,
        LIFT,
    }

    public State state;

    DcMotorEx intakeMotor;
    CRServo liftServo;
    Servo hardstop;

    public Intake(HardwareMap hardwareMap){
        liftServo = hardwareMap.get(CRServo.class, "lift");
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        hardstop = hardwareMap.get(Servo.class, "hardstop");

        state = State.IDLE;
    }

    @Override
    public void init(){
        intakeMotor.setPower(0);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void stop(){
        liftServo.setPower(0);
        intakeMotor.setPower(0);
        state = State.IDLE;
    }

    public void in(){
        liftServo.setPower(1);
        intakeMotor.setPower(1);

        state = State.IN;
    }

    public void spit(){
        liftServo.setPower(-1);
        intakeMotor.setPower(-1);

        state = State.SPIT;
    }

    public void lift(){
        liftServo.setPower(1);
        intakeMotor.setPower(0);

        state = State.LIFT;
    }

    public void stopLift(){
        liftServo.setPower(0);

        state = State.IDLE;
    }

    public void extendHardstop(){
        hardstop.setPosition(1);
    }

    public void retractHardstop(){
        hardstop.setPosition(0);
    }

    @Override
    public void update(){}

    @Override
    public void telemetry(Telemetry tele){
        tele.addData("Intake State:", state);
    }
}

