package org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public class RetractHardstopCommand extends InstantCommand {
    public RetractHardstopCommand(){
        super(
                () -> {
                    Robot.getInstance().intake.retractHardstop();
                }
        );

        addRequirements(Robot.getInstance().intake);
    }
}
