package org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public class LiftBallCommand extends InstantCommand {
    public LiftBallCommand(){
        super(
                () -> {
                    Robot.getInstance().intake.lift();
                }
        );

        addRequirements(Robot.getInstance().intake);
    }
}
