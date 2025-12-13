package org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter;

import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.util.GreenSubsystem;

public class Kicker implements GreenSubsystem, Subsystem {
    public Servo kickerServo;

    public Kicker(HardwareMap hardwareMap){
        kickerServo = hardwareMap.get(Servo.class, "kicker");
    }

    @Override
    public void init() {
        down();
    }

    public void up() {
        kickerServo.setPosition(0);
    }

    public void down(){
        kickerServo.setPosition(1);
    }

    @Override
    public void update() {}

    @Override
    public void telemetry(Telemetry telemetry) {

    }
}
