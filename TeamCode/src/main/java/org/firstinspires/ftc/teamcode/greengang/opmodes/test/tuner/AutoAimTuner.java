package org.firstinspires.ftc.teamcode.greengang.opmodes.test.tuner;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commonA.intakeA.IntakeA;
import org.firstinspires.ftc.teamcode.commonA.outtakeA.OuttakeA;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;

@Config
@TeleOp
public class AutoAimTuner extends GreenLinearOpMode {
    private OuttakeA shooter;
    private DcMotorEx intake;
    private Servo hardstop;

    public static double target = 0;
    private boolean toggle = false;
    @Override
    public void initialize() {
        shooter = new OuttakeA(hardwareMap);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        hardstop = hardwareMap.get(Servo.class, "hardstop");

        addStickyG1();
    }

    @Override
    public void telemetry(Telemetry tele) {

    }

    @Override
    public void periodic() {
        shooter.setTarget_Vel(target);

        if (toggle) {
            shooter.setOuttakeState(OuttakeA.OuttakeState.SPIN_UP);
        } else{
            shooter.setOuttakeState(OuttakeA.OuttakeState.OFF);
        }

        if(gamepad1.a){
            toggle = true;
        }

        if(gamepad1.b){
            toggle = false;
        }


        if(gamepad1.right_bumper){
            intake.setPower(1);
        } else{
            intake.setPower(0);
        }

        if(gamepad1.left_bumper){
            hardstop.setPosition(0);
        } else{

            hardstop.setPosition(1);
        }

        shooter.runFlywheelPID();

        telemetry.addData("Hardstop pos", hardstop.getPosition());
        telemetry.addData("Velocity", shooter.getFlyVel());
    }
}
