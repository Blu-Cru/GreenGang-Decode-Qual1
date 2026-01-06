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
    }

    public State state;

    DcMotorEx intakeMotor;
    Servo hardstop;

    public Intake(HardwareMap hardwareMap){
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        hardstop = hardwareMap.get(Servo.class, "hardstop");

        state = State.IDLE;
    }

    @Override
    public void init(){
        intakeMotor.setPower(0);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        retractHardstop();
    }

    public void stop(){
        intakeMotor.setPower(0);
        state = State.IDLE;
    }

    public void in(){
        intakeMotor.setPower(1);

        state = State.IN;
    }

    public void spit(){
        intakeMotor.setPower(-1);

        state = State.SPIT;
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

