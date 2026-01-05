package org.firstinspires.ftc.teamcode.greengang.common.util;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commonA.drivetrainA.Drivetrain;
import org.firstinspires.ftc.teamcode.commonA.outtakeA.ShooterA;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.drive.DrivetrainOLD;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.intake.Intake;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter.Kicker;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter.Shooter;

import java.util.ArrayList;

public class Robot {
    private static Robot instance;
    HardwareMap hardwareMap; // reference to hardware

    // all subsystems
    public Drivetrain drivetrain;
    public Shooter sh;
    public ShooterA shooterA;
    public Intake intake;
    public Kicker kicker;

    ArrayList<GreenSubsystem> subsystems;

    public static Robot getInstance() {
        if(instance == null) {
            instance = new Robot();
        }
        return instance;
    }

    private Robot(){
        subsystems = new ArrayList<>();
    }

    public Robot create(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        return this;
    }

    // initializes subsystems
    public void init() {
        for(GreenSubsystem subsystem : subsystems) {
            subsystem.init();
        }
    }
//
//    public PinPointLocalizer getPpl() {
//        ppl = new PinPointLocalizer(hardwareMap);
//        subsystems.add(ppl);
//        return ppl;
//    }

    public Shooter addShooter(){
        sh = new Shooter(hardwareMap);
        subsystems.add(sh);

        return sh;
    }

    public Intake addIntake(){
        intake = new Intake(hardwareMap);
        subsystems.add(intake);

        return intake;
    }

    public Kicker addKicker(){
        kicker = new Kicker(hardwareMap);
        subsystems.add(kicker);

        return kicker;
    }

    public Drivetrain addDrivetrain() {
        drivetrain = new Drivetrain(hardwareMap);
        subsystems.add(drivetrain);

        return drivetrain;
    }

    public ShooterA addShooterA() {
        shooterA = new ShooterA(hardwareMap);
        subsystems.add(shooterA);

        return shooterA;
    }

    public void telemetry(Telemetry telemetry) {
        for(GreenSubsystem subsystem : subsystems) {
            subsystem.telemetry(telemetry);
        }
    }

    public void update(){
        for (GreenSubsystem subsystem : subsystems) {
            subsystem.update();
        }
    }

    // call this after every op mode
    public static void kill() {
        instance = null;
    }
}
