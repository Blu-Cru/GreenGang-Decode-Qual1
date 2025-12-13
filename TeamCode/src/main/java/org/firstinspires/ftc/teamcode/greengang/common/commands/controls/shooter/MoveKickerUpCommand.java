package org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public class MoveKickerUpCommand extends InstantCommand {
    public MoveKickerUpCommand() {
        super(
                () -> {
                    Robot.getInstance().kicker.up();
                }
        );

        addRequirements(Robot.getInstance().kicker);
    }
}
