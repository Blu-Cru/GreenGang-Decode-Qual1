package org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.util.AprilTagTargeting;
import org.firstinspires.ftc.teamcode.greengang.common.util.Globals;
import org.firstinspires.ftc.teamcode.greengang.common.util.GreenSubsystem;

@Config
public class Shooter implements GreenSubsystem, Subsystem {
    public static double kP = 0.0025, kI = 0.005, kD = 0.0001, ff = 0.00037;
    public static double target, velocity = 0;

    //tune these
    public static double DEFAULT_VELOCITY = 4000;
    public static double DEFAULT_HOOD_POSITION = 0;

    public enum State {
        IDLE,
        AUTO,
        MANUAL,
        REVERSE,
    }

    public State state;

    DcMotorEx flywheel1;
    DcMotorEx flywheel2;
    PIDController controller;
    Servo hoodServo;

    public Shooter(HardwareMap hardwareMap){
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        hoodServo = hardwareMap.get(Servo.class, "hood");

        controller = new PIDController(kP, kI, kD);

        state = State.IDLE;
    }

    @Override
    public void init(){
        flywheel1.setPower(0);
        flywheel2.setPower(0);

        flywheel1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        flywheel2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        hoodServo.setPosition(DEFAULT_HOOD_POSITION);
    }


    public void setTargetVelocity(double t){
        state = State.AUTO;
        target = t;
    }

    public void updatePid(){
        controller.setPID(kP, kI, kD);
    }

    public void setShooterState(State s){
        state = s;
    }

    public void stop(){
        state = State.IDLE;
    }

    public double getFlywheelVelocity(){
        return velocity;
    }

    public double getFlywheelTargetVelocity(){
        return target;
    }

    @Override
    public void update(){
        velocity = flywheel1.getVelocity();

        double power = 0;
        switch(state){
            case IDLE:
                power = 0;
                break;
            case AUTO:
                power = controller.calculate(velocity, target) + ff  * target;
                break;
            case MANUAL:
                target = DEFAULT_VELOCITY;
                power = controller.calculate(velocity, target) + ff  * target;
                break;
            case REVERSE:
                power = -1;
                break;
        }

        flywheel1.setPower(power);
        flywheel2.setPower(power);
    }

    @Override
    public void telemetry(Telemetry tele){
        tele.addData("velocity", velocity);
        tele.addData("target", target);
    }
}

