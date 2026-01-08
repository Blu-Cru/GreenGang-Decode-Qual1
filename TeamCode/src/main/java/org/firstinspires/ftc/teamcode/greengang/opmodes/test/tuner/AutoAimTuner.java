package org.firstinspires.ftc.teamcode.greengang.opmodes.test.tuner;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.ExtendHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.RetractHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;

@Config
@TeleOp
public class AutoAimTuner extends GreenLinearOpMode {
    public static double target = 0;
    private boolean toggle = false;
    @Override
    public void initialize() {
        addShooter();
        addStickyG1();
    }

    @Override
    public void telemetry(Telemetry tele) {
        tele.addData("Velocity", shooter.getFlywheelVelocity());
        tele.addData("Target Velocity", target);
    }

    @Override
    public void periodic() {


        if (toggle) {
            new RetractHardstopCommand().schedule();
            shooter.setTargetVelocity(target);
        } else{
            new ExtendHardstopCommand().schedule();
            shooter.setShooterState(Shooter.State.IDLE);
        }

        if(stickyG1.a){
            toggle = !toggle;
        }


        if(gamepad1.left_trigger > 0.3){
            intake.in();
        } else{
            intake.stop();
        }
    }
}
