package org.firstinspires.ftc.teamcode.greengang.common.commands.shoot;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.ExtendHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.RetractHardstopCommand;

public class ShootCommand extends SequentialCommandGroup {
    public ShootCommand(){
        super(
                new SequentialCommandGroup(
                        /*

                        (3x
                        spin tube wheel
                        kick
                        )

                        */
                        new RetractHardstopCommand(),
//                        new WaitUntilCommand(() -> {
//                            double current = Robot.getInstance().shooter.getFlywheelVelocity();
//                            double target = Robot.getInstance().shooter.getFlywheelTargetVelocity();
//
//                            return Math.abs(current - target) <= target * 0.1;
//                        }),
                        new WaitCommand(1600),
                        new KickBallCommand(),
                        new ExtendHardstopCommand()
                )
        );
    }
}
