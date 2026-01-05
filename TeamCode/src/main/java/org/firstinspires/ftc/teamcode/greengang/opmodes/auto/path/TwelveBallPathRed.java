package org.firstinspires.ftc.teamcode.greengang.opmodes.auto.path;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;

@Autonomous(group = "paths")
public class TwelveBallPathRed extends GreenLinearOpMode {
    private static double deg(double d) {
        return Math.toRadians(d);
    }

    private Action path;

    @Override
    public void initialize() {
        addDrivetrain();
        addShooter();
        addStickyG1();
        addStickyG2();
        addKicker();
        addIntake();

        Pose2d startPose = new Pose2d(63, -24, 0);

        drivetrainOLD.drive.localizer.setPose(startPose);

        path = drivetrainOLD.drive.actionBuilder(startPose)
                .setReversed(true)
                .setTangent(deg(-45))
                .splineToSplineHeading(
                        new Pose2d(-24, 24, deg(-225)),
                        deg(-45)
                )

                // .waitSeconds(1)

                // shoot

                .setReversed(false)
                .setTangent(deg(0))
                .splineToSplineHeading(
                        new Pose2d(-12, 30, deg(90)),
                        deg(90)
                )

                // .waitSeconds(1)

                // intake
                .lineToY(50)

                .setReversed(true)
                .setTangent(deg(-112))
                .splineToSplineHeading(
                        new Pose2d(-24, 24, deg(-225)),
                        deg(-112)
                )
                //.waitSeconds(1)

                // shoot

                .setReversed(false)
                .setTangent(deg(0))
                .splineToSplineHeading(
                        new Pose2d(14, 35, deg(90)),
                        deg(90)
                )

                // .waitSeconds(1)

                // intake
                .lineToY(55)

                .setReversed(true)
                .setTangent(deg(-60))
                .splineToSplineHeading(
                        new Pose2d(-24, 24, deg(-225)),
                        deg(-139)
                )

                //.waitSeconds(1)

                .setReversed(false)
                .setTangent(deg(0))
                .splineToSplineHeading(
                        new Pose2d(36, 40, deg(90)),
                        deg(90)
                )

                //.waitSeconds(1)

                .lineToY(55)

                //.waitSeconds(1)

                .setReversed(true)
                .setTangent(deg(-130))
                .splineToSplineHeading(
                        new Pose2d(-24, 24, deg(-225)),
                        deg(-153)
                )
                .build();
    }

    @Override
    public void onStart() {
        Actions.runBlocking(path);
    }

    @Override
    public void telemetry(Telemetry tele) {
        Pose2d p = drivetrainOLD.drive.localizer.getPose();
        tele.addData("x", p.position.x);
        tele.addData("y", p.position.y);
        tele.addData("heading (deg)", Math.toDegrees(p.heading.toDouble()));
    }
}