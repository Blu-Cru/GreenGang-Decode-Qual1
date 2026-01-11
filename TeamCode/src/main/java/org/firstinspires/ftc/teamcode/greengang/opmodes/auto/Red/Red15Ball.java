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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;

@Autonomous(name = "Auto Red 15 Ball")
public class Red15Ball extends OpMode {
    public static class Paths {

        public PathChain Preload, ShootToSpike2, Spike2ToShoot, ShootToGateCycle, GateCycleToShoot;
        public PathChain ShootToSpike1, Spike1ToShoot, ShootToSpike3, Spike3ToShoot, Park;

        // X reflected (144 - x), Y same, Heading reflected (180 - theta)
        public Pose startPose = new Pose(126, 120, Math.toRadians(36));
        public Pose shootPose = new Pose(97.5, 97.0, Math.toRadians(38));
        public Pose spike2Entry = new Pose(102.81, 56, Math.toRadians(0));
        public Pose spike2PickUp = new Pose(134.77, 54.5, Math.toRadians(0));
        public Pose gateCycle = new Pose(135.1, 57.6, Math.toRadians(24.8));
        public Pose spike1Entry = new Pose(102, 84, Math.toRadians(0));
        public Pose spike1PickUp = new Pose(130.1, 83.7, Math.toRadians(-1));
        public Pose spike3Entry = new Pose(85, 36, Math.toRadians(0));
        public Pose spike3PickUp = new Pose(135, 34.2, Math.toRadians(-2));
        public Pose parkPose = new Pose(123, 100, Math.toRadians(45));

        public Paths(Follower follower) {
            Preload = follower.pathBuilder()
                    .addPath(new BezierLine(startPose, shootPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                    .build();

            ShootToSpike2 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, new Pose(89.202, 60), spike2Entry))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), spike2Entry.getHeading())
                    .addPath(new BezierLine(spike2Entry, spike2PickUp))
                    .setLinearHeadingInterpolation(spike2Entry.getHeading(), spike2PickUp.getHeading())
                    .build();

            Spike2ToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(spike2PickUp, new Pose(70, 48), shootPose))
                    .setLinearHeadingInterpolation(spike2Entry.getHeading(), shootPose.getHeading())
                    .build();

            ShootToGateCycle = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, new Pose(80, gateCycle.getY()), gateCycle))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), gateCycle.getHeading())
                    .build();

            GateCycleToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(gateCycle, new Pose(80, 61.620), shootPose))
                    .setLinearHeadingInterpolation(gateCycle.getHeading(), shootPose.getHeading())
                    .build();

            ShootToSpike1 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, new Pose(80, 82.399), spike1Entry))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), spike1Entry.getHeading())
                    .addPath(new BezierLine(spike1Entry, spike1PickUp))
                    .setLinearHeadingInterpolation(spike1Entry.getHeading(), spike1PickUp.getHeading())
                    .build();

            Spike1ToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(spike1PickUp, new Pose(80, 86.996), shootPose))
                    .setLinearHeadingInterpolation(spike1PickUp.getHeading(), shootPose.getHeading())
                    .build();

            ShootToSpike3 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, new Pose(67.688, 35.508), spike3Entry))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), spike3Entry.getHeading())
                    .addPath(new BezierLine(spike3Entry, spike3PickUp))
                    .setLinearHeadingInterpolation(spike3Entry.getHeading(), spike3PickUp.getHeading())
                    .build();

            Spike3ToShoot = follower.pathBuilder()
                    .addPath(new BezierCurve(spike3PickUp, new Pose(78.905, 63.459), shootPose))
                    .setLinearHeadingInterpolation(spike3PickUp.getHeading(), shootPose.getHeading())
                    .build();

            Park = follower.pathBuilder()
                    .addPath(new BezierLine(shootPose, parkPose))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                    .build();
        }
    }

    private IntakeA intake;
    private OuttakeA outtake;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Paths paths;
    private double gateCount = 0;

    public enum PathState { PRELOAD, SHOOT, GET_SPIKE2, SPIKE2_SHOOT, GET_GATE_CYCLE, GATE_CYCLE_SHOOT, GET_SPIKE1, SPIKE1_SHOOT, GET_SPIKE3, SPIKE3_SHOOT, PARK, END }
    PathState pathState;
    double start_shoot_time = -0.3;
    private String prevState;

    public void statePathUpdate() {
        switch (pathState) {
            case PRELOAD:
                prevState = "Preload";
                // Conditional flipped: Blue checked if < 18.5, Red checks if > 125.5
                if (follower.getPose().getX() > 125.5) {
                    follower.followPath(paths.Preload, true);
                    outtake.setHoodPosition(0);
                    outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                }
                // Red transition check: x < 108 (reflected from 36)
                if (follower.getPose().getX() < 108) {
                    setPathState(PathState.SHOOT);
                }
                break;

            case SHOOT:
                if (pathTimer.getElapsedTimeSeconds()<0.25) outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                else outtake.setOuttakeState(OuttakeA.OuttakeState.SPIN_UP);

                if (pathTimer.getElapsedTimeSeconds()< 0.34 && Objects.equals(prevState, "Preload")) outtake.setTarget_Vel(1720);
                else if (Objects.equals(prevState, "Preload")) outtake.setTarget_Vel(1580);
                else outtake.setTarget_Vel(Storage.autoAimFlywheelPow);

                if (pathTimer.getElapsedTimeSeconds() >= start_shoot_time + 1.55) {
                    outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                    if (Objects.equals(prevState, "Preload")) setPathState(PathState.GET_SPIKE2);
                    else if (Objects.equals(prevState, "GetSpike2")) setPathState(PathState.GET_GATE_CYCLE);
                    else if (Objects.equals(prevState, "GetGateCycle1")) setPathState(PathState.GET_SPIKE1);
                    else if (Objects.equals(prevState, "GetSpike1")) setPathState(PathState.GET_SPIKE3);
                    else if (Objects.equals(prevState, "GetSpike3")) setPathState(PathState.PARK);
                }
                break;

            case GET_SPIKE2:
                follower.setMaxPower(0.8);
                prevState="GetSpike2";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) follower.followPath(paths.ShootToSpike2, true);
                // Transition check reflected: Blue < 10 -> Red > 134
                if (follower.getPose().getX() > 125.5 || pathTimer.getElapsedTimeSeconds()>4) {
                    follower.setMaxPower(1.0);
                    setPathState(PathState.SPIKE2_SHOOT);
                }
                break;

            case SPIKE2_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) follower.followPath(paths.Spike2ToShoot);
                if (follower.getPose().getY()>88 || pathTimer.getElapsedTimeSeconds()>4) setPathState(PathState.SHOOT);
                break;

            case GET_GATE_CYCLE:
                prevState="GetGateCycle1";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.ShootToGateCycle, true);
                }
                // Check reflected: Blue (9.3 < x < 33) -> Red (111 < x < 134.7)
                if (follower.getPose().getX() > 111 && follower.getPose().getX() < 134.7) follower.setMaxPower(0.7);

                if (pathTimer.getElapsedTimeSeconds() > 4 || pathTimer.getElapsedTimeSeconds() > 7) {
                    follower.setMaxPower(1.0);
                    gateCount += 1;
                    setPathState(PathState.GATE_CYCLE_SHOOT);
                }
                break;

            case GATE_CYCLE_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) follower.followPath(paths.GateCycleToShoot);
                // Check reflected: Blue > 45.7 -> Red < 98.3
                if (follower.getPose().getX() < 98.3 || pathTimer.getElapsedTimeSeconds() > 5) setPathState(PathState.SHOOT);
                break;

            case GET_SPIKE1:
                prevState="GetSpike1";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) follower.followPath(paths.ShootToSpike1, true);
                if (follower.getPose().getX() > 100.5) follower.setMaxPower(0.65); // Reflected from 43.5
                if (follower.getPose().getX() > 121.5 || pathTimer.getElapsedTimeSeconds()>4) {
                    follower.setMaxPower(1.0);
                    setPathState(PathState.SPIKE1_SHOOT);
                }
                break;

            case SPIKE1_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) follower.followPath(paths.Spike1ToShoot);
                if (follower.getPose().getX() < 99.5 || pathTimer.getElapsedTimeSeconds()>4) setPathState(PathState.SHOOT);
                break;

            case GET_SPIKE3:
                prevState="GetSpike3";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) follower.followPath(paths.ShootToSpike3, true);
                if (follower.getPose().getX() > 124.5 || pathTimer.getElapsedTimeSeconds()>4) setPathState(PathState.SPIKE3_SHOOT);
                break;

            case SPIKE3_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) follower.followPath(paths.Spike3ToShoot);
                if (follower.getPose().getX() < 98 || pathTimer.getElapsedTimeSeconds()>4) setPathState(PathState.SHOOT);
                break;

            case PARK:
                intake.setIntakeState(IntakeA.IntakeState.IDLE);
                outtake.setOuttakeState(OuttakeA.OuttakeState.OFF);
                if (pathTimer.getElapsedTimeSeconds()<0.1) follower.followPath(paths.Park, true);
                break;

            case END:
                intake.setIntakeState(IntakeA.IntakeState.IDLE);
                outtake.setOuttakeState(OuttakeA.OuttakeState.OFF);
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
        paths = new Paths(follower);
        follower.setPose(paths.startPose);
        pathState = PathState.PRELOAD;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.PRELOAD);
        intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN);
        outtake.setTarget_Vel(1820);
        outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        outtake.periodic();
        intake.periodic();
        telemetry.update();
    }

    @Override
    public void stop() {
        if (intake != null) { intake.setIntakeState(IntakeA.IntakeState.IDLE); intake.periodic(); }
        if (outtake != null) { outtake.setOuttakeState(OuttakeA.OuttakeState.OFF); outtake.periodic(); }
        Storage.isRed = true;
        Storage.lastPose = follower.getPose();
    }
}



/*
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
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;

@Autonomous(name = "Auto Red 15 Ball")
public class Red15Ball extends OpMode {
    public static class Paths {

        public PathChain Preload;
        public PathChain ShootToSpike2;
        public PathChain Spike2ToShoot;
        public PathChain ShootToGateCycle;
        public PathChain GateCycleToShoot;
        public PathChain ShootToSpike1;
        public PathChain Spike1ToShoot;
        public PathChain ShootToSpike3;
        public PathChain Spike3ToShoot;
        public PathChain Park;

        public Pose startPose = new Pose(126, 120, Math.toRadians(36));
        public Pose shootPose = new Pose(97.5, 97.000, Math.toRadians(38));
        public Pose spike2Entry = new Pose(102.81, 60, Math.toRadians(0));
        public Pose spike2PickUp = new Pose(134.77, 58.4, Math.toRadians(-2));
        public Pose gateCycle = new Pose(135.1, 57.3, Math.toRadians(25.8));
        public Pose spike1Entry = new Pose(102, 84, Math.toRadians(0));
        public Pose spike1PickUp = new Pose(130.1, 84, Math.toRadians(-1));
        public Pose spike3Entry = new Pose(103.729, 36, Math.toRadians(0));
        public Pose spike3PickUp = new Pose(135, 34.7, Math.toRadians(-2));
        public Pose parkPose = new Pose(123, 105, Math.toRadians(45));

        public Paths(Follower follower) {

            Preload = follower
                    .pathBuilder()
                    .addPath(new BezierLine(startPose, shootPose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                    .build();

            ShootToSpike2 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(shootPose, new Pose(89.202, 70.446), spike2Entry))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), spike2Entry.getHeading())
                    .addPath(new BezierLine(spike2Entry, spike2PickUp))
                    .setLinearHeadingInterpolation(spike2Entry.getHeading(), spike2PickUp.getHeading())
                    .build();

            Spike2ToShoot = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(spike2PickUp, new Pose(79, 48), shootPose))
                    .setLinearHeadingInterpolation(spike2PickUp.getHeading(), shootPose.getHeading())
                    .build();

            ShootToGateCycle = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(shootPose, new Pose(91.409, gateCycle.getY()), gateCycle))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), gateCycle.getHeading())
                    .build();

            GateCycleToShoot = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(gateCycle, new Pose(91.593, 61.620), shootPose))
                    .setLinearHeadingInterpolation(gateCycle.getHeading(), shootPose.getHeading())
                    .build();

            ShootToSpike1 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(shootPose, new Pose(90.857, 82.399), spike1Entry))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), spike1Entry.getHeading())
                    .addPath(new BezierLine(spike1Entry, spike1PickUp))
                    .setLinearHeadingInterpolation(spike1Entry.getHeading(), spike1PickUp.getHeading())
                    .build();

            Spike1ToShoot = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(spike1PickUp, new Pose(101.339, 86.996), shootPose))
                    .setLinearHeadingInterpolation(spike1PickUp.getHeading(), shootPose.getHeading())
                    .build();

            ShootToSpike3 = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(shootPose, new Pose(67.688, 35.508), spike3Entry))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), spike3Entry.getHeading())
                    .addPath(new BezierLine(spike3Entry, spike3PickUp))
                    .setLinearHeadingInterpolation(spike3Entry.getHeading(), spike3PickUp.getHeading())
                    .build();

            Spike3ToShoot = follower
                    .pathBuilder()
                    .addPath(new BezierCurve(spike3PickUp, new Pose(78.905, 63.459), shootPose))
                    .setLinearHeadingInterpolation(spike3PickUp.getHeading(), shootPose.getHeading())
                    .build();

            Park = follower
                    .pathBuilder()
                    .addPath(new BezierLine(shootPose, parkPose))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), parkPose.getHeading())
                    .build();
        }
    }

    private IntakeA intake;
    private OuttakeA outtake;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Paths paths;

    public enum PathState {
        PRELOAD, SHOOT, GET_SPIKE2, SPIKE2_SHOOT, GET_GATE_CYCLE, GATE_CYCLE_SHOOT, GET_SPIKE1, SPIKE1_SHOOT, GET_SPIKE3, SPIKE3_SHOOT, PARK, END
    }

    PathState pathState;
    double start_shoot_time = 0.02;
    private String prevState;

    public void statePathUpdate() {
        switch (pathState) {
            case PRELOAD:
                prevState = "Preload";
                if (follower.getPose().getX() > 125.5) {
                    follower.followPath(paths.Preload, true);
                    outtake.setHoodPosition(0.022);
                }
                if (follower.getPose().getX() < 99.4) {
                    setPathState(PathState.SHOOT);
                }
                break;

            case SHOOT:
                outtake.setOuttakeState(OuttakeA.OuttakeState.SPIN_UP);

                if (pathTimer.getElapsedTimeSeconds() > start_shoot_time && pathTimer.getElapsedTimeSeconds() < start_shoot_time + 1.25) {
                    if (pathTimer.getElapsedTimeSeconds() < start_shoot_time + 1.03) {
                        outtake.setTarget_Vel(2160);
                        outtake.setHoodPosition(0.021);
                    } else {
                        outtake.setTarget_Vel(1970);
                        outtake.setHoodPosition(0.021);
                    }
                    intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN_SHOOT);
                } else if (pathTimer.getElapsedTimeSeconds() >= start_shoot_time + 1.25 && pathTimer.getElapsedTimeSeconds() < start_shoot_time + 1.71) {
                    outtake.setTarget_Vel(1955);
                    outtake.setOuttakeState(OuttakeA.OuttakeState.FIRE);
                    intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN);
                } else if (pathTimer.getElapsedTimeSeconds() >= start_shoot_time + 2.2) {
                    outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                    if (Objects.equals(prevState, "Preload")) {
                        setPathState(PathState.GET_SPIKE2);
                    } else if (Objects.equals(prevState, "GetSpike2")) {
                        setPathState(PathState.GET_GATE_CYCLE);
                    } else if (Objects.equals(prevState, "GetGateCycle1")) {
                        setPathState(PathState.GET_SPIKE1);
                    } else if (Objects.equals(prevState, "GetSpike1")) {
                        setPathState(PathState.GET_SPIKE3);
                    } else if (Objects.equals(prevState, "GetSpike3")) {
                        setPathState(PathState.PARK);
                    }
                }
                break;

            case GET_SPIKE2:
                prevState="GetSpike2";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.ShootToSpike2, true);
                }
                if (follower.getPose().getX() > 134) {
                    setPathState(PathState.SPIKE2_SHOOT);
                }
                break;

            case SPIKE2_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.Spike2ToShoot);
                }
                if (follower.getPose().getY() > 95) {
                    setPathState(PathState.SHOOT);
                }
                break;

            case GET_GATE_CYCLE:
                prevState="GetGateCycle1";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.ShootToGateCycle, true);
                }
                if (follower.getPose().getX() > 111 && follower.getPose().getX() < 134.7) {
                    follower.setMaxPower(0.65);
                }
                if ((follower.getPose().getX() > 134.7 && pathTimer.getElapsedTimeSeconds() > 2) || pathTimer.getElapsedTimeSeconds() > 4) {
                    follower.setMaxPower(1.0);
                    setPathState(PathState.GATE_CYCLE_SHOOT);
                }
                break;

            case GATE_CYCLE_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.GateCycleToShoot);
                }
                if (follower.getPose().getX() < 98.8) {
                    setPathState(PathState.SHOOT);
                }
                break;

            case GET_SPIKE1:
                prevState="GetSpike1";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.ShootToSpike1, true);
                }
                if (follower.getPose().getX() > 100.5) {
                    follower.setMaxPower(0.65);
                }
                if (follower.getPose().getX() > 127.3) {
                    follower.setMaxPower(1.0);
                    setPathState(PathState.SPIKE1_SHOOT);
                }
                break;

            case SPIKE1_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.Spike1ToShoot);
                }
                if (follower.getPose().getX() < 99.1) {
                    setPathState(PathState.SHOOT);
                }
                break;

            case GET_SPIKE3:
                prevState="GetSpike3";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.ShootToSpike3, true);
                }
                if (follower.getPose().getX() > 134) {
                    setPathState(PathState.SPIKE3_SHOOT);
                }
                break;

            case SPIKE3_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.Spike3ToShoot);
                }
                if (follower.getPose().getX() < 98) {
                    setPathState(PathState.SHOOT);
                }
                break;

            case PARK:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.Park);
                }
                if (follower.getPose().getY() > 116) {
                    setPathState(PathState.END);
                }
                break;

            case END:
                intake.setIntakeState(IntakeA.IntakeState.IDLE);
                outtake.setOuttakeState(OuttakeA.OuttakeState.OFF);
                requestOpModeStop();
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
        paths = new Paths(follower);
        follower.setPose(new Pose(126, 120, Math.toRadians(36)));
        pathState = PathState.PRELOAD;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.PRELOAD);
        intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN);
        outtake.setTarget_Vel(2050.0);
        outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
    }

    @Override
    public void loop() {
        follower.update();
        statePathUpdate();
        outtake.periodic();
        intake.periodic();

        telemetry.addData("State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.update();
    }
    @Override
    public void stop() {
        Storage.isRed=true;
        Storage.lastPose = follower.getPose();
    }
}*/
