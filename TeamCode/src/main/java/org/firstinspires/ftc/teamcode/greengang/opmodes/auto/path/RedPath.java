package org.firstinspires.ftc.teamcode.greengang.opmodes.auto.path;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Actions;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;

@Autonomous(group = "paths")
public class RedPath extends GreenLinearOpMode {

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

        startPose = new Pose2d(63, -24, 0);

        // Set initial pose
        drivetrain.drive.localizer.setPose(startPose);

        // Build path ONCE
        path = drivetrain.drive.actionBuilder(startPose)
                .splineToLinearHeading(
                        new Pose2d(0, 0, Math.toRadians(225)),
                        Math.toRadians(225)
                )
                .waitSeconds(2)
                .splineToLinearHeading(
                        new Pose2d(36, -22, Math.toRadians(-90)),
                        Math.toRadians(-90)
                )
                .lineToY(-52)
                .waitSeconds(1)
                .splineToLinearHeading(
                        new Pose2d(0, 0, Math.toRadians(225)),
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
        boolean running = path.run(null);
        if(!running){
            path = null;
            drivetrain.idle();
        }
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