package org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter;

import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

import com.arcrobotics.ftclib.command.InstantCommand;

public class StartFlywheelCommand extends InstantCommand {
    public StartFlywheelCommand(){
        super(
                () -> {
                    Robot.getInstance().sh.startFlywheel();
                }
        );

        addRequirements(Robot.getInstance().sh);
    }
}
