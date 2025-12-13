package org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public class StartIntakeCommand extends InstantCommand {
    public StartIntakeCommand(){
        super(
                () -> {
                    Robot.getInstance().intake.in();
                }
        );

        addRequirements(Robot.getInstance().intake);
    }
}
