package org.firstinspires.ftc.teamcode.greengang.opmodes.auto.Red; // Changed package to Red

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

@Autonomous(group = "auto", preselectTeleOp = "RCSMTeleOp")
public class RedPathPark extends GreenLinearOpMode { // Changed class name

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

        // X-AXIS REFLECTION: Y -> -Y, Heading -> -Heading
        // Original: startPose = new Pose2d(-50, -50, deg(225));
        startPose = new Pose2d(-50, 50, deg(-225)); // Hardcoded Reflection
        drivetrain.drive.localizer.setPose(startPose);

        path = drivetrain.drive.actionBuilder(startPose)

                // Original: .setReversed(true) -> SWAPPED: .setReversed(false)
                .setReversed(false)

                // Original: deg(45) -> Hardcoded: deg(-45)
                .setTangent(deg(-45))

                // Original: new Pose2d(-24, -24, deg(225)), deg(45)
                // Reflected: (-24, 24, -225 deg), -45 deg
                .splineToSplineHeading(
                        new Pose2d(-24, 24, deg(-225)),
                        deg(-45)
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

                // Original: .setReversed(false) -> SWAPPED: .setReversed(true)
                .setReversed(true)

                // Original: deg(0) -> Hardcoded: deg(0)
                .setTangent(deg(0))

                // Original: new Pose2d(-12, -22, deg(-90)), deg(-90)
                // Reflected: (-12, 22, 90 deg), 90 deg
                .splineToSplineHeading(
                        new Pose2d(-12, 30, deg(90)),
                        deg(90)
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

                // Original: .lineToY(-50) -> Hardcoded: .lineToY(50)
                .lineToY(67)
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new StopIntakeCommand(),
                                        new StartFlywheelCommand(),
                                        new RetractHardstopCommand()
                                )
                        )
                )

                // Original: .setReversed(true) -> SWAPPED: .setReversed(false)
                .setReversed(false)

                // Original: deg(112) -> Hardcoded: deg(-112)
                .setTangent(deg(-112))

                // Original: new Pose2d(-24, -24, deg(225)), deg(112)
                // Reflected: (-24, 24, -225 deg), -112 deg
                .splineToSplineHeading(
                        new Pose2d(-24, 24, deg(-225)),
                        deg(-112)
                )

                // shoot
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ShootCommand()
                                )
                        )
                )

                // Original: .setReversed(false) -> SWAPPED: .setReversed(true)
                .setReversed(true)

                // Original: deg(0) -> Hardcoded: deg(0)
                .setTangent(deg(0))

                // Original: new Pose2d(14, -22, deg(-90)), deg(-90)
                // Reflected: (14, 22, 90 deg), 90 deg
                .splineToSplineHeading(
                        new Pose2d(10, 30, deg(90)),
                        deg(90)
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

                // Original: .lineToY(-55) -> Hardcoded: .lineToY(55)
                .lineToY(72)
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new StopIntakeCommand(),
                                        new StartFlywheelCommand(),
                                        new RetractHardstopCommand()
                                )
                        )
                )

                // Original: .setReversed(true) -> SWAPPED: .setReversed(false)
                .setReversed(false)

                // Original: deg(60) -> Hardcoded: deg(-60)
                .setTangent(deg(-60))

                // Original: new Pose2d(-24, -24, deg(225)), deg(139)
                // Reflected: (-24, 24, -225 deg), -139 deg
                .splineToSplineHeading(
                        new Pose2d(-24, 24, deg(-225)),
                        deg(-139)
                )

                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ShootCommand()
                                )
                        )
                )

                // Original: .setReversed(false) -> SWAPPED: .setReversed(true)
                .setReversed(true)

                // Original: deg(0) -> Hardcoded: deg(0)
                .setTangent(deg(0))

                // Original: new Pose2d(36, -22, deg(-90)), deg(-90)
                // Reflected: (36, 22, 90 deg), 90 deg
                .splineToSplineHeading(
                        new Pose2d(34, 30, deg(90)),
                        deg(90)
                )

                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new ExtendHardstopCommand(),
                                        new IntakeCommand()
                                )
                        )
                )

                // Original: .lineToY(-55) -> Hardcoded: .lineToY(55)
                .lineToY(72)
                .stopAndAdd(
                        new CommandToAction(
                                new SequentialCommandGroup(
                                        new StopIntakeCommand(),
                                        new StartFlywheelCommand(),
                                        new RetractHardstopCommand()
                                )
                        )
                )

                // Original: .setReversed(true) -> SWAPPED: .setReversed(false)
                .setReversed(false)

                // Original: deg(130) -> Hardcoded: deg(-130)
                .setTangent(deg(-130))

                // Original: new Pose2d(-24, -24, deg(225)), deg(153)
                // Reflected: (-24, 24, -225 deg), -153 deg
                .splineToSplineHeading(
                        new Pose2d(-24, 24, deg(-225)),
                        deg(-153)
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