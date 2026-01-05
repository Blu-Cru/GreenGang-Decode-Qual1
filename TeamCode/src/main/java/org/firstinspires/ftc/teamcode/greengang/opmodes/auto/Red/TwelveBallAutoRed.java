package org.firstinspires.ftc.teamcode.greengang.opmodes.auto.Red;

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
import org.firstinspires.ftc.teamcode.greengang.common.commands.controls.shooter.StopShooterCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.intake.IntakeCommand;
import org.firstinspires.ftc.teamcode.greengang.common.commands.shoot.ShootCommand;
import org.firstinspires.ftc.teamcode.greengang.common.util.CommandToAction;
import org.firstinspires.ftc.teamcode.greengang.common.util.Robot;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;

@Autonomous(group = "auto")
public class TwelveBallAutoRed extends GreenLinearOpMode {

    private Action path;

    private static double deg(double d) {
        return Math.toRadians(d);
    }

    CommandToAction startFlywheel =
            new CommandToAction(
                    new SequentialCommandGroup(
                            new StartFlywheelCommand(),
                            new RetractHardstopCommand()
                    )
            );

    CommandToAction intake =
            new CommandToAction(
                    new SequentialCommandGroup(
                            new ExtendHardstopCommand()
                    )
            );

    CommandToAction shoot =
            new CommandToAction(
                    new ShootCommand()
            );

    @Override
    public void initialize() {
        addDrivetrain();
        addShooter();
        addStickyG1();
        addStickyG2();
        addKicker();
        addIntake();

        // Reflected start pose
        Pose2d startPose = new Pose2d(-50, 50, deg(-225));
        drivetrainOLD.drive.localizer.setPose(startPose);

        path = drivetrainOLD.drive.actionBuilder(startPose)
                .setReversed(true)
                .setTangent(deg(-45))
                .splineToSplineHeading(
                        new Pose2d(-24, 24, deg(-225)),
                        deg(-45)
                )

                // shoot
                .stopAndAdd(new CommandToAction(
                        new SequentialCommandGroup(
                                new IntakeCommand(),
                                new ShootCommand()
                        )
                ))

                .setReversed(false)
                .setTangent(deg(0))
                .splineToSplineHeading(
                        new Pose2d(-12, 30, deg(90)),
                        deg(90)
                )

                // intake
                .lineToY(50)

                .setReversed(true)
                .setTangent(deg(-112))
                .splineToSplineHeading(
                        new Pose2d(-24, 24, deg(-225)),
                        deg(-112)
                )

                // shoot
                .stopAndAdd(new CommandToAction(
                        new ShootCommand()
                ))

                .setReversed(false)
                .setTangent(deg(0))
                .splineToSplineHeading(
                        new Pose2d(14, 35, deg(90)),
                        deg(90)
                )

                // intake
                .lineToY(55)

                .setReversed(true)
                .setTangent(deg(-60))
                .splineToSplineHeading(
                        new Pose2d(-24, 24, deg(-225)),
                        deg(-139)
                )

                .stopAndAdd(new CommandToAction(
                        new ShootCommand()
                ))

                .setReversed(false)
                .setTangent(deg(0))
                .splineToSplineHeading(
                        new Pose2d(36, 40, deg(90)),
                        deg(90)
                )

                .lineToY(55)

                .setReversed(true)
                .setTangent(deg(-130))
                .splineToSplineHeading(
                        new Pose2d(-24, 24, deg(-225)),
                        deg(-153)
                )

                // shoot
                .stopAndAdd(new CommandToAction(
                        new ShootCommand()
                ))

                // kill
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new StopIntakeCommand(),
                                        new StopShooterCommand(),
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
        Pose2d p = drivetrainOLD.drive.localizer.getPose();

        tele.addData("x", p.position.x);
        tele.addData("y", p.position.y);
        tele.addData("heading (deg)", Math.toDegrees(p.heading.toDouble()));
    }
}
