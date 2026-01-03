package org.firstinspires.ftc.teamcode.greengang.opmodes.test.tuner;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commonA.intakeA.IntakeA;
import org.firstinspires.ftc.teamcode.commonA.outtakeA.OuttakeA;

@Config
@TeleOp
public class AutoAimTuner extends CommandOpMode {
    private OuttakeA shooter;
    private IntakeA intake;

    public static double target = 0;
    @Override
    public void initialize() {
        shooter = new OuttakeA(hardwareMap);
        intake = new IntakeA(hardwareMap);
    }

    @Override
    public void run() {
        super.run();

        shooter.setTarget_Vel(target);
        shooter.setOuttakeState(OuttakeA.OuttakeState.FIRE);

        if(gamepad1.right_trigger > 0.001){
            intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN_SHOOT);
        }
    }
}
