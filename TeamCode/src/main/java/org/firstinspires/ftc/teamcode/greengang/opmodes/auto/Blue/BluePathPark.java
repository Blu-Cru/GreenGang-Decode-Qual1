package org.firstinspires.ftc.teamcode.greengang.opmodes.auto.Blue;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.ExtendHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.RetractHardstopCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.intake.StopIntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.MoveKickerDownCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StartFlywheelCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StopLiftingBallCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.shoot.ShootCommand;
import org.firstinspires.ftc.teamcode.greengang.common.util.CommandToAction;
import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;

@Autonomous(group = "auto")
public class BluePathPark extends GreenLinearOpMode {

    private Action path;
    private Pose2d startPose;

    private static double deg(double d) {
        return Math.toRadians(d);
    }

    @Override
    public void initialize() {
        addDrivetrain();
        addShooter();
        addStickyG1();
        addStickyG2();
        addKicker();
        addIntake();

        startPose = new Pose2d(-50, -50, deg(225));
        drivetrain.drive.localizer.setPose(startPose);

        path = drivetrain.drive.actionBuilder(startPose)

                .setReversed(true)
                .setTangent(deg(45))
                .splineToSplineHeading(
                        new Pose2d(-24, -24, deg(225)),
                        deg(45)
                )

                // shoot
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new StartFlywheelCommand(),
                                        new ShootCommand()
                                )
                        )
                )

                .setReversed(false)
                .setTangent(deg(0))
                .splineToSplineHeading(
                        new Pose2d(-12, -22, deg(-90)),
                        deg(-90)
                )

                // intake
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ExtendHardstopCommand(),
                                        new IntakeCommand()
                                )
                        )
                )
                .lineToY(-50)
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
                .setTangent(deg(112))
                .splineToSplineHeading(
                        new Pose2d(-24, -24, deg(225)),
                        deg(112)
                )

                // shoot
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ShootCommand()
                                )
                        )
                )

                .setReversed(false)
                .setTangent(deg(0))
                .splineToSplineHeading(
                        new Pose2d(14, -22, deg(-90)),
                        deg(-90)
                )

                // intake
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ExtendHardstopCommand(),
                                        new IntakeCommand()
                                )
                        )
                )
                .lineToY(-55)
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
                .setTangent(deg(60))
                .splineToSplineHeading(
                        new Pose2d(-24, -24, deg(225)),
                        deg(139)
                )

                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ShootCommand()
                                )
                        )
                )

                .setReversed(false)
                .setTangent(deg(0))
                .splineToSplineHeading(
                        new Pose2d(36, -22, deg(-90)),
                        deg(-90)
                )

                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ExtendHardstopCommand(),
                                        new IntakeCommand()
                                )
                        )
                )
                .lineToY(-55)
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
                .setTangent(deg(130))
                .splineToSplineHeading(
                        new Pose2d(-24, -24, deg(225)),
                        deg(153)
                )

                // shoot
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
