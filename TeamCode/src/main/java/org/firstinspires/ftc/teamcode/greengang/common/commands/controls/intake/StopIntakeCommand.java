package org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public class StopIntakeCommand extends InstantCommand {
    public StopIntakeCommand(){
        super(
                () -> {
                    Robot.getInstance().intake.stop();
                }
        );

        addRequirements(Robot.getInstance().intake);
    }
}
