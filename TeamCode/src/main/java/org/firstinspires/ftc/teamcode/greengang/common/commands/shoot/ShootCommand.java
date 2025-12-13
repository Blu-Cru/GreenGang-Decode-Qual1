package org.firstinspires.ftc.teamcode.greengang.common.commands.shoot;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.LiftBallCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StartFlywheelCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StopShooterCommand;
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
                        new LiftBallCommand(),
                        new WaitUntilCommand(() -> {
                            double current = Robot.getInstance().sh.getFlywheelVelocity();
                            double target = Robot.getInstance().sh.getFlywheelTargetVelocity();

                            return Math.abs(current - target) <= target * 0.1;
                        }).withTimeout(3), // 1.5 second timeout to prevent infinite hang
                        new KickBallCommand()
                )
        );
    }
}
