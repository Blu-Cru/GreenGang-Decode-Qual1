package org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter;

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
    public static double DEFAULT_VELOCITY = 2100;
    public static double DEFAULT_HOOD_POSITION = 0.083;

    public enum State {
        IDLE,
        AUTO,
        MANUAL,
    }

    public State state;

    DcMotorEx flywheel;
    PIDController controller;
    AprilTagTargeting targeting;
    Servo hoodServo;

    public boolean started = false;

    public Shooter(HardwareMap hardwareMap){
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        hoodServo = hardwareMap.get(Servo.class, "hood");

        controller = new PIDController(kP, kI, kD);

        state = State.IDLE;
    }

    @Override
    public void init(){
        flywheel.setPower(0);

        flywheel.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        hoodServo.setPosition(DEFAULT_HOOD_POSITION);
    }

    public void setTargeting(AprilTagTargeting targeting) {
        this.targeting = targeting;
    }

    public void setTargetVelocity(double t){
        state = State.AUTO;
        target = t;
    }

    public void updatePid(){
        controller.setPID(kP, kI, kD);
    }

    public void startFlywheel(){
        //update later
        if(Globals.autoAimEnabled){
            double d = targeting.getDistanceToGoal();

            target = ShooterData.velocityFromDistance(d);
            setTargetVelocity(target);

            state = State.AUTO;
        } else{
            state = State.MANUAL;
        }
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

    public void updateHood() {
        if (state == State.AUTO) {
            double d = targeting.getDistanceToGoal();
            double hoodPosition = ShooterData.hoodFromDistance(d);

            hoodServo.setPosition(hoodPosition);
        } else {
            hoodServo.setPosition(DEFAULT_HOOD_POSITION);
        }
    }

    @Override
    public void update(){
        if(!started){
            startFlywheel();
            started = true;
        }

        velocity = flywheel.getVelocity();

        updateHood();

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
        }

        flywheel.setPower(power);
    }

    @Override
    public void telemetry(Telemetry tele){
        tele.addData("velocity", velocity);
        tele.addData("target", target);
    }
}

