
package org.firstinspires.ftc.teamcode.greengang.opmodes.auto.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.commonA.intakeA.IntakeA;
import org.firstinspires.ftc.teamcode.commonA.outtakeA.OuttakeA;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Auto 12 Ball")
public class Blue12BallGate extends OpMode {
    private IntakeA intake;
    private OuttakeA outtake;
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        PRELOAD,
        SHOOT,
        GET_SPIKE,
        GO_TO_SHOOT,
        PARK,
        END
    }

    PathState pathState;

    // --- POSES ---
    private final Pose startPose = new Pose(18, 120, Math.toRadians(144));
    private final Pose shootPose = new Pose(46.5, 97, Math.toRadians(142));

    // Parking Position
    private final Pose parkPose = new Pose(38.5, 72, Math.toRadians(180));

    private final Pose spike1Entry = new Pose(33, 84, Math.toRadians(0));
    private final Pose spike1PickUp = new Pose(16, 83.3, Math.toRadians(180));
    private final Pose spike1Clearance = new Pose(12.2, 77, Math.toRadians(90));

    private final Pose spike2Entry = new Pose(33, 58, Math.toRadians(180));
    private final Pose spike2PickUp = new Pose(8, 58, Math.toRadians(180));

    private final Pose spike3Entry = new Pose(33, 36, Math.toRadians(180));
    private final Pose spike3PickUp = new Pose(8, 36, Math.toRadians(180));

    // --- PATHS ---
    private PathChain startToShoot, shootToSpike, spikeToShoot, shootToPark;

    double start_shoot_time = 0.02;
    private double targ_vel = 2050;
    private int spikeCount = 0;

    public void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        // Park Path
        shootToPark = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(60, 97, Math.toRadians(90)), parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void buildSpikePath(int count) {
        if (count == 1) {
            shootToSpike = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, new Pose(60, 84, Math.toRadians(180)), spike1Entry))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                    .addPath(new BezierLine(spike1Entry, spike1PickUp))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            spikeToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(spike1PickUp,new Pose(26, 83), spike1Clearance))
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(93))
                    .addPath(new BezierCurve(
                            spike1Clearance,
                            new Pose(25.3, 83.5),
                            new Pose(48, 86, Math.toRadians(148)),
                            shootPose
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(93), shootPose.getHeading())
                    .build();
        } else {
            Pose entry = (count == 2) ? spike2Entry : spike3Entry;
            Pose pickup = (count == 2) ? spike2PickUp : spike3PickUp;

            shootToSpike = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, new Pose(60, entry.getY(), Math.toRadians(180)), entry))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(180))
                    .addPath(new BezierLine(entry, pickup))
                    .setConstantHeadingInterpolation(Math.toRadians(180))
                    .build();

            spikeToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(pickup, new Pose(62, 81, Math.toRadians(142)), shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(180), shootPose.getHeading())
                    .build();
        }
    }

    public void statePathUpdate() {
        switch (pathState) {
            case PRELOAD:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(startToShoot, true);
                    outtake.setHoodPosition(0.022);
                }
                if (follower.getPose().getX() > 44.5 || !follower.isBusy()) {
                    setPathState(PathState.SHOOT);
                }
                break;

            case SHOOT:
                outtake.setOuttakeState(OuttakeA.OuttakeState.REV);

                if (pathTimer.getElapsedTimeSeconds() > start_shoot_time && pathTimer.getElapsedTimeSeconds() < start_shoot_time + 1.25) {
                    if (pathTimer.getElapsedTimeSeconds() < start_shoot_time + 1.03) {
                        outtake.setTarget_Vel(2160);
                        outtake.setHoodPosition(0.021);
                    } else {
                        outtake.setTarget_Vel(1970);
                        outtake.setHoodPosition(0.021);
                    }
                    intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN_SHOOT);
                }
                else if (pathTimer.getElapsedTimeSeconds() >= start_shoot_time + 1.25 && pathTimer.getElapsedTimeSeconds() < start_shoot_time + 1.71) {
                    outtake.setTarget_Vel(1955);
                    outtake.setOuttakeState(OuttakeA.OuttakeState.FIRE);
                    intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN);
                }
                else if (pathTimer.getElapsedTimeSeconds() >= start_shoot_time + 2.2) {
                    outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                    spikeCount++;
                    if (spikeCount <= 3) {
                        buildSpikePath(spikeCount);
                        setPathState(PathState.GET_SPIKE);
                    } else {
                        setPathState(PathState.PARK);
                    }
                }
                break;

            case GET_SPIKE:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(shootToSpike, false);
                    intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN);
                }
                if (follower.getPose().getX() < 17.5 || !follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(PathState.GO_TO_SHOOT);
                }
                break;

            case GO_TO_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.3) {
                    follower.followPath(spikeToShoot, true);
                    telemetry.addLine("going to shoot path");
                }
                else {
                    double triggerX = (spikeCount == 1) ? 46.0 : 43.5;

                    if (follower.getPose().getX() > triggerX || !follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4.0) {
                        setPathState(PathState.SHOOT);
                    }
                }


                break;

            case PARK:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(shootToPark, true);
                    outtake.setOuttakeState(OuttakeA.OuttakeState.OFF);
                    intake.setIntakeState(IntakeA.IntakeState.IDLE);
                }
                if (!follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 3.0) {
                    setPathState(PathState.END);
                }
                break;

            case END:
                telemetry.addLine("Auto Complete - Parked");
                break;
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        intake = new IntakeA(hardwareMap);
        outtake = new OuttakeA(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setPose(startPose);
        pathState = PathState.PRELOAD;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.PRELOAD);
        intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN);
        outtake.setTarget_Vel(targ_vel);
        outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        outtake.periodic();
        intake.periodic();

        telemetry.addData("State", pathState);
        telemetry.addData("Spike Count", spikeCount);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }
}