package org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public class MoveKickerDownCommand extends InstantCommand {
    public MoveKickerDownCommand() {
        super(
                () -> {
                    Robot.getInstance().kicker.down();
                }
        );

        addRequirements(Robot.getInstance().kicker);
    }
}
