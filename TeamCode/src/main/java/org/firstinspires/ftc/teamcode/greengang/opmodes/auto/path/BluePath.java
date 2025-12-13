package org.firstinspires.ftc.teamcode.greengang.opmodes.auto.path;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;

@Autonomous(group = "paths")
public class BluePath extends GreenLinearOpMode {

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
                .splineToLinearHeading(
                        new Pose2d(-24, -24, Math.toRadians(225)),
                        Math.toRadians(225)
                )
                .waitSeconds(2)
                .splineToLinearHeading(
                        new Pose2d(36, -22, Math.toRadians(-90)),
                        Math.toRadians(-90)
                )
                .lineToY(-52)
                .waitSeconds(1)
                .setReversed(true)
                .splineToLinearHeading(
                        new Pose2d(-24, -24, Math.toRadians(225)),
                        Math.toRadians(225)
                )
                .waitSeconds(2)
                .splineToLinearHeading(
                        new Pose2d(39, -33, Math.toRadians(-90)),
                        Math.toRadians(-90)
                )
                .build();
    }

    @Override
    public void onStart() {
        Actions.runBlocking(path);
    }

    @Override
    public void periodic(){
    }

    @Override
    public void telemetry(Telemetry tele) {
        Pose2d p = drivetrain.drive.localizer.getPose();
        tele.addData("x", p.position.x);
        tele.addData("y", p.position.y);
        tele.addData("heading (deg)", Math.toDegrees(p.heading.toDouble()));
    }
}