package org.firstinspires.ftc.teamcode.greengang.common.commands.controls.outtake;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

public class OuttakeCommand extends InstantCommand {
    public OuttakeCommand() {
        super(
                () -> {
                    Robot.getInstance().intake.spit();
                }
        );

        addRequirements(Robot.getInstance().intake);
    }
}
