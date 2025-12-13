package org.firstinspires.ftc.teamcode.greengang.common.commands.intake;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.ExtendHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.StartIntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.LiftBallCommand;
import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public class IntakeCommand extends SequentialCommandGroup {
    public IntakeCommand(){
        super(
                new SequentialCommandGroup(
                        new StartIntakeCommand()
                )
        );
    }
}
