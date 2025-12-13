package org.firstinspires.ftc.teamcode.greengang.common.commands.shoot;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.MoveKickerDownCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.MoveKickerUpCommand;

public class KickBallCommand extends SequentialCommandGroup {
    public KickBallCommand(){
        super(
                new SequentialCommandGroup(
                        new MoveKickerUpCommand(),
                        new WaitCommand(1500),
                        new MoveKickerDownCommand()
                )
        );
    }
}
