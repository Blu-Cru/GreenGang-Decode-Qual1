package org.firstinspires.ftc.teamcode.greengang.opmodes.auto.Blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.ExtendHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.RetractHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.MoveKickerDownCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StartFlywheelCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StopLiftingBallCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StopShooterCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.shoot.ShootCommand;
import org.firstinspires.ftc.teamcode.greengang.common.util.CommandToAction;
import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;

@Autonomous(group = "auto")
public class BluePathPark extends GreenLinearOpMode {

    private Action path;
    private Pose2d startPose;

    @Override
    public void initialize() {
        addDrivetrain();
        addShooter();
        addStickyG1();
        addStickyG2();
        addKicker();
        addIntake();

        startPose = new Pose2d(63, -24, Math.toRadians(-90));
        drivetrain.drive.localizer.setPose(startPose);

        path = drivetrain.drive.actionBuilder(startPose)

                .setTangent(Math.toRadians(160))
                .splineToSplineHeading(
                        new Pose2d(-24, -24, Math.toRadians(225)),
                        Math.toRadians(180)
                )

                //shoot
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new StartFlywheelCommand(),
                                        new ShootCommand()
                                )
                        )
                )


                // Hub -> -12,-22: end facing -90, approach downward so lineToY is clean
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(-12, -22, Math.toRadians(-90)),
                        Math.toRadians(-90)
                )

                //intake
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ExtendHardstopCommand(),
                                        new IntakeCommand()
                                )
                        )
                )
                .lineToY(-55 )
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new StopIntakeCommand(),
                                        new StartFlywheelCommand(),
                                        new RetractHardstopCommand()
                                )
                        )
                )

                .setReversed(true)

                // Return -> Hub: keep facing 225, but arc UP into hub (end tangent ~135)
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(
                        new Pose2d(-24, -24, Math.toRadians(225)),
                        Math.toRadians(135)
                )

                //shoot

                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ShootCommand()
                                )
                        )
                )


                // Hub -> +12,-22: same idea
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(12, -22, Math.toRadians(-90)),
                        Math.toRadians(-90)
                )
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ExtendHardstopCommand(),
                                        new IntakeCommand()
                                )
                        )
                )
                .lineToY(-55 )
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new StopIntakeCommand(),
                                        new StartFlywheelCommand(),
                                        new RetractHardstopCommand()
                                )
                        )
                )

                .setReversed(true)

                // Return -> Hub: arc UP into hub again
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(
                        new Pose2d(-24, -24, Math.toRadians(225)),
                        Math.toRadians(135)
                )

                //shoot

                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ShootCommand()
                                )
                        )
                )

                // Hub -> 36,-22
                .setTangent(Math.toRadians(0))
                .splineToSplineHeading(
                        new Pose2d(36, -22, Math.toRadians(-90)),
                        Math.toRadians(-90)
                )

                //intake
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ExtendHardstopCommand(),
                                        new IntakeCommand()
                                )
                        )
                )
                .lineToY(-55 )
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new StopIntakeCommand(),
                                        new StartFlywheelCommand(),
                                        new RetractHardstopCommand()
                                )
                        )
                )

                .setReversed(true)

                // Return -> Hub
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(
                        new Pose2d(-24, -24, Math.toRadians(225)),
                        Math.toRadians(135)
                )

                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ShootCommand()
                                )
                        )
                )

                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new StopIntakeCommand(),
                                        new StopLiftingBallCommand(),
                                        new MoveKickerDownCommand()
                                )
                        )
                )


                .build();
    }

    @Override
    public void onStart() {
        Actions.runBlocking(
                packet -> {
                    Robot.getInstance().update();
                    return path.run(packet);
                }
        );
    }

    @Override
    public void telemetry(Telemetry tele) {
        Pose2d p = drivetrain.drive.localizer.getPose();
        tele.addData("x", p.position.x);
        tele.addData("y", p.position.y);
        tele.addData("heading (deg)", Math.toDegrees(p.heading.toDouble()));
    }
}