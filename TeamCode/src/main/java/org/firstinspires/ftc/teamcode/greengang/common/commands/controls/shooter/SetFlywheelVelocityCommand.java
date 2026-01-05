package org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public class SetFlywheelVelocityCommand extends InstantCommand {
    public SetFlywheelVelocityCommand(double velocity){
        super(
                () -> {
                    Robot.getInstance().sh.setTargetVelocity(velocity);
                }
        );

        addRequirements(Robot.getInstance().sh);
    }
}
