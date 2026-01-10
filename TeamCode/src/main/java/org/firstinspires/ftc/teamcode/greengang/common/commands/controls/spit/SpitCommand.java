package org.firstinspires.ftc.teamcode.greengang.common.commands.controls.spit;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public class SpitCommand extends InstantCommand {
    public SpitCommand() {
        super(
                () -> {
                    Robot.getInstance().intake.spit();
                }
        );

        addRequirements(Robot.getInstance().intake);
    }
}
