package org.firstinspires.ftc.teamcode.commonA.commandsA;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public class ShooterSwitchStateCommand extends InstantCommand {

    public ShooterSwitchStateCommand(ShooterA.State targetState) {
        super(
                () -> {
                    Robot.getInstance().shooterA.setState(targetState);
                }
        );

        addRequirements(Robot.getInstance().shooterA);
    }
}