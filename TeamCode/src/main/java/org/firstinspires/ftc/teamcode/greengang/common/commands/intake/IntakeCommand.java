package org.firstinspires.ftc.teamcode.greengang.common.commands.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.StartIntakeCommand;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(){
        super(
                new SequentialCommandGroup(
                        new StartIntakeCommand()
                )
        );
    }
}
