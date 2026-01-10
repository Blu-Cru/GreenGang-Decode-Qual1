package org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter;

import org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

import com.arcrobotics.ftclib.command.InstantCommand;

public class SwitchFlywheelStateCommand extends InstantCommand {
    public SwitchFlywheelStateCommand(Shooter.State state){
        super(
                () -> {
                    Robot.getInstance().shooter.setShooterState(state);
                }
        );

        addRequirements(Robot.getInstance().shooter);
    }
}
