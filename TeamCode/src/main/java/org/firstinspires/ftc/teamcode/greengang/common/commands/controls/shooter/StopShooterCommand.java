package org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public class StopShooterCommand extends InstantCommand {
    public StopShooterCommand(){
        super(
                () -> {
                    Robot.getInstance().shooter.stop();
                }
        );

        addRequirements(Robot.getInstance().shooter);
    }
}
