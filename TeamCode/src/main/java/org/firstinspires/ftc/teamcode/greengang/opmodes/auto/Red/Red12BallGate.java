package org.firstinspires.ftc.teamcode.greengang.opmodes.auto.Red;

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
import org.firstinspires.ftc.teamcode.greengang.GlobalsStorage.Storage;
import org.firstinspires.ftc.teamcode.greengang.opmodes.auto.Blue.Blue12BallGate;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Red Auto 12 Ball")
public class Red12BallGate extends OpMode {
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

    // --- MIRRORED POSES ---
    private final Pose startPose = new Pose(126, 120, Math.toRadians(36));
    private final Pose shootPose = new Pose(97.5, 97, Math.toRadians(38));

    private final Pose parkPose = new Pose(123, 105, Math.toRadians(45));

    private final Pose spike1Entry = new Pose(111, 84, Math.toRadians(180));
    private final Pose spike1PickUp = new Pose(128, 83.3, Math.toRadians(0));
    private final Pose spike1Clearance = new Pose(131.8, 77, Math.toRadians(87));

    private final Pose spike2Entry = new Pose(111, 57, Math.toRadians(0));
    private final Pose spike2PickUp = new Pose(136, 57, Math.toRadians(0));

    private final Pose spike3Entry = new Pose(111, 36, Math.toRadians(0));
    private final Pose spike3PickUp = new Pose(136, 36, Math.toRadians(0));

    private PathChain startToShoot, shootToSpike, spikeToShoot, shootToPark;

    double start_shoot_time = 0.02;
    private double targ_vel = 2050;
    private int spikeCount = 0;

    public void buildPaths() {
        startToShoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

        shootToPark = follower.pathBuilder()
                .addPath(new BezierCurve(shootPose, new Pose(84, 97, Math.toRadians(90)), parkPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                .build();
    }

    public void buildSpikePath(int count) {
        if (count == 1) {
            shootToSpike = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, new Pose(84, 84, Math.toRadians(0)), spike1Entry))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(0))
                    .addPath(new BezierLine(spike1Entry, spike1PickUp))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            spikeToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(spike1PickUp, new Pose(118, 83), spike1Clearance))
                    .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(87))
                    .addPath(new BezierCurve(
                            spike1Clearance,
                            new Pose(118.7, 83.5),
                            new Pose(96, 86, Math.toRadians(32)),
                            shootPose
                    ))
                    .setLinearHeadingInterpolation(Math.toRadians(87), shootPose.getHeading())
                    .build();
        } else {
            Pose entry = (count == 2) ? spike2Entry : spike3Entry;
            Pose pickup = (count == 2) ? spike2PickUp : spike3PickUp;

            shootToSpike = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, new Pose(84, entry.getY(), Math.toRadians(0)), entry))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), Math.toRadians(0))
                    .addPath(new BezierLine(entry, pickup))
                    .setConstantHeadingInterpolation(Math.toRadians(0))
                    .build();

            spikeToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(pickup, new Pose(82, 81, Math.toRadians(38)), shootPose))
                    .setLinearHeadingInterpolation(Math.toRadians(0), shootPose.getHeading())
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
                // MIRRORED LOGIC: Check if X is less than shoot target
                if (follower.getPose().getX() < 99.5 || !follower.isBusy()) {
                    setPathState(PathState.SHOOT);
                }
                break;

            case SHOOT:
                outtake.setOuttakeState(OuttakeA.OuttakeState.SPIN_UP);
                outtake.setTarget_Vel(1600);

                if (pathTimer.getElapsedTimeSeconds() >= start_shoot_time + 1.9) {
                    outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                    spikeCount++;
                    if (spikeCount <= 3) {
                        buildSpikePath(spikeCount);
                        setPathState(Red12BallGate.PathState.GET_SPIKE);
                    } else {
                        setPathState(Red12BallGate.PathState.PARK);
                    }
                }
                break;

            case GET_SPIKE:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(shootToSpike, false);
                    intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN);
                }
                // MIRRORED LOGIC: Check if X is greater than pickup target
                if (follower.getPose().getX() > 126.5 || !follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 2.5) {
                    setPathState(PathState.GO_TO_SHOOT);
                }
                break;

            case GO_TO_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.3) {
                    follower.followPath(spikeToShoot, true);
                }
                else {
                    // MIRRORED LOGIC: trigger value is 144 - triggerX
                    double triggerX = (spikeCount == 1) ? 98.0 : 100.5;

                    if (follower.getPose().getX() < triggerX || !follower.isBusy() || pathTimer.getElapsedTimeSeconds() > 4.0) {
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
    @Override
    public void stop() {
        Storage.isRed=true;
        Storage.lastPose = follower.getPose();
    }
}