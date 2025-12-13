package org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public class ExtendHardstopCommand extends InstantCommand {
    public ExtendHardstopCommand(){
        super(
                () -> {
                    Robot.getInstance().intake.extendHardstop();
                }
        );

        addRequirements(Robot.getInstance().intake);
    }
}
