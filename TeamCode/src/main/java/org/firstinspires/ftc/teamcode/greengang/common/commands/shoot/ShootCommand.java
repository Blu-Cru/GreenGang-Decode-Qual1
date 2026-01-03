package org.firstinspires.ftc.teamcode.greengang.common.commands.shoot;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.ExtendHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.RetractHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.LiftBallCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StartFlywheelCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StopLiftingBallCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StopShooterCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.subsystems.shooter.Shooter;
import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;

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
//                            double current = Robot.getInstance().sh.getFlywheelVelocity();
//                            double target = Robot.getInstance().sh.getFlywheelTargetVelocity();
//
//                            return Math.abs(current - target) <= target * 0.1;
//                        }),
                        new WaitCommand(1750),
                        new KickBallCommand(),
                        new ExtendHardstopCommand()
                )
        );
    }
}
