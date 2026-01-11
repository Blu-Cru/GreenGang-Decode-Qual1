package org.firstinspires.ftc.teamcode.greengang.opmodes.auto.Red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commonA.intakeA.IntakeA;
import org.firstinspires.ftc.teamcode.commonA.outtakeA.OuttakeA;
import org.firstinspires.ftc.teamcode.greengang.GlobalsStorage.Storage;
import org.firstinspires.ftc.teamcode.greengang.common.util.Alliance;
import org.firstinspires.ftc.teamcode.greengang.common.util.Globals;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;
import org.firstinspires.ftc.teamcode.greengang.opmodes.auto.Blue.Blue18Ball;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;


@Autonomous(name = "Auto Red 18 Ball", preselectTeleOp = "Tele")
public class Red18Ball extends GreenLinearOpMode {
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
        public Pose startPose = new Pose(144 - 18, 120, Math.toRadians(180 - 144));
        public Pose shootPose = new Pose(144 - 48, 100.000, Math.toRadians(180 - 144+0.2));
        public Pose spike2Entry = new Pose(144 - 60, 59.5, Math.toRadians(180 - 180));
        public Pose spike2PickUp = new Pose(144 - 9.32, 57.13, Math.toRadians(180 - 180));
        public Pose gateCycle = new Pose(144 - 9.6, 57.9, Math.toRadians(180 - 154.3));
        public Pose spike1Entry = new Pose(144 - 60, 85, Math.toRadians(180 - 180));
        public Pose spike1PickUp = new Pose(144 - 14.3, 86.4, Math.toRadians(180 - 180));
        public Pose spike3Entry = new Pose(85, 40.7, Math.toRadians(0));
        public Pose spike3PickUp = new Pose(135, 38, Math.toRadians(-2));
        public Pose parkPose = new Pose(144 - 21, 95, Math.toRadians(180 - 135));


        public Paths(Follower follower) {

            Preload = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(startPose, shootPose)
                    )
                    .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                    .build();

            ShootToSpike2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    shootPose,
                                    new Pose(124 - 54.798, 65),
                                    spike2Entry
                            )
                    )
                    .setLinearHeadingInterpolation(shootPose.getHeading(), spike2Entry.getHeading())
                    .addPath(
                            new BezierLine(spike2Entry, spike2PickUp)
                    )
                    .setLinearHeadingInterpolation(spike2Entry.getHeading(), spike2PickUp.getHeading())
                    .build();


            Spike2ToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    spike2PickUp,
                                    new Pose(124 - 65, 48),
                                    shootPose
                            )
                    )
                    .setLinearHeadingInterpolation(spike2Entry.getHeading(), shootPose.getHeading())
                    .build();

            ShootToGateCycle = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    shootPose,
                                    new Pose(124 - 52.591, gateCycle.getY()),
                                    gateCycle
                            )
                    )
                    .setLinearHeadingInterpolation(shootPose.getHeading(), gateCycle.getHeading())
                    .build();

            GateCycleToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    gateCycle,
                                    new Pose(124 - 52.407, 61.620),
                                    shootPose
                            )
                    )
                    .setLinearHeadingInterpolation(gateCycle.getHeading(), shootPose.getHeading())
                    .build();



            ShootToSpike1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    shootPose,
                                    new Pose(124 - 53.143, 90),
                                    spike1Entry
                            )
                    )
                    .setLinearHeadingInterpolation(shootPose.getHeading(), spike1Entry.getHeading())
                    .addPath(
                            new BezierLine(spike1Entry, spike1PickUp)
                    )
                    .setLinearHeadingInterpolation(spike1Entry.getHeading(), spike1PickUp.getHeading())
                    .build();



            Spike1ToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    spike1PickUp,
                                    new Pose(124 - 42.661, 86.996),
                                    shootPose
                            )
                    )
                    .setLinearHeadingInterpolation(spike1PickUp.getHeading(), shootPose.getHeading())
                    .build();
            ShootToSpike3 = follower.pathBuilder()
                    .addPath(new BezierCurve(shootPose, new Pose(50, 52.5), spike3Entry))
                    .setLinearHeadingInterpolation(shootPose.getHeading(), spike3Entry.getHeading())
                    .addPath(new BezierLine(spike3Entry, spike3PickUp))
                    .setLinearHeadingInterpolation(spike3Entry.getHeading(), spike3PickUp.getHeading())
                    .build();



            Spike3ToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    spike3PickUp,
                                    new Pose(124 - 65.095, 63.459),
                                    shootPose
                            )
                    )
                    .setLinearHeadingInterpolation(spike3PickUp.getHeading(), shootPose.getHeading())
                    .build();

            Park = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(shootPose, parkPose)
                    )
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

    private boolean toggle = false;

    public enum PathState {
        PRELOAD,
        SHOOT,
        GET_SPIKE2,
        SPIKE2_SHOOT,
        GET_GATE_CYCLE,
        GATE_CYCLE_SHOOT,
        GET_SPIKE1,
        SPIKE1_SHOOT,
        GET_SPIKE3,
        SPIKE3_SHOOT,
        PARK,
        END
    }

    PathState pathState;

    double start_shoot_time = -0.3;

    private String prevState;

    public void statePathUpdate() {
        switch (pathState) {
            case PRELOAD:
                prevState = "Preload";
                if (follower.getPose().getX() > (144 - 18.5)) {
                    follower.followPath(paths.Preload, true);
                    outtake.setHoodPosition(0);
                    outtake.setOuttakeState(OuttakeA.OuttakeState.REV);

                }
                if (follower.getPose().getX() < (144 - 36)) {
                    setPathState(PathState.SHOOT);
                }
                break;

            case SHOOT:
                if (pathTimer.getElapsedTimeSeconds()<0.25) outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                else outtake.setOuttakeState(OuttakeA.OuttakeState.SPIN_UP);
                if (pathTimer.getElapsedTimeSeconds()< 0.34 && Objects.equals(prevState, "Preload")) {
                    outtake.setTarget_Vel(1710);
                }
                else if (Objects.equals(prevState, "Preload")) outtake.setTarget_Vel(1580);
                else outtake.setTarget_Vel(Storage.autoAimFlywheelPow-30);


                if (pathTimer.getElapsedTimeSeconds() >= start_shoot_time + 1.55) {
                    outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                    if (Objects.equals(prevState, "Preload")) {
                        setPathState(PathState.GET_SPIKE2);
                    } else if (Objects.equals(prevState, "GetSpike2")) {
                        setPathState(PathState.GET_GATE_CYCLE);
                    } else if (Objects.equals(prevState, "GetGateCycle1") && gateCount==1) {
                        setPathState(PathState.GET_GATE_CYCLE);

                    } else if (Objects.equals(prevState, "GetGateCycle1") && gateCount==2) {
                        setPathState(PathState.GET_SPIKE1);
                    } else if (Objects.equals(prevState, "GetSpike1")) {
                        setPathState(PathState.GET_SPIKE3);
                    } else if (Objects.equals(prevState, "GetSpike3")) {
                        setPathState(PathState.PARK);
                    }

                }
                break;

            case GET_SPIKE2:
                follower.setMaxPower(0.8);
                prevState="GetSpike2";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.ShootToSpike2, true);

                }
                if (follower.getPose().getX() > (144 - 27) || pathTimer.getElapsedTimeSeconds()>4) {
                    follower.setMaxPower(1.0);
                    setPathState(PathState.SPIKE2_SHOOT);
                }
                break;

            case SPIKE2_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.Spike2ToShoot);
                }
                if (follower.getPose().getY()>88|| pathTimer.getElapsedTimeSeconds()>4) { //y val could be decreased
                    setPathState(PathState.SHOOT);
                }
                break;

            case GET_GATE_CYCLE:
                prevState="GetGateCycle1";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.setMaxPower(1.0);
                    follower.followPath(paths.ShootToGateCycle, true);

                }
                if (follower.getPose().getX() > (144 - 33) && follower.getPose().getX() < (144 - 9.3)) {
                    follower.setMaxPower(0.7);
                }
                if (pathTimer.getElapsedTimeSeconds()>3.5 && gateCount==1) {
                    follower.setMaxPower(1.0);
                    gateCount+=1;
                    setPathState(PathState.GATE_CYCLE_SHOOT);
                }
                if (pathTimer.getElapsedTimeSeconds()>3.7 && gateCount==0|| pathTimer.getElapsedTimeSeconds()>7) {
                    follower.setMaxPower(1.0);
                    gateCount+=1;
                    setPathState(PathState.GATE_CYCLE_SHOOT);
                }
                break;

            case GATE_CYCLE_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.GateCycleToShoot);
                }
                if (follower.getPose().getX() < (144 - 45.7) || pathTimer.getElapsedTimeSeconds()>5) {
                    setPathState(PathState.SHOOT);
                }
                break;
            case GET_SPIKE1:
                prevState="GetSpike1";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.ShootToSpike1, true);

                }
                if (follower.getPose().getX() > (144 - 43.5)) {
                    follower.setMaxPower(0.65);
                }
                if (follower.getPose().getX() > (144 - 29.2) || pathTimer.getElapsedTimeSeconds()>4) {
                    follower.setMaxPower(1.0);
                    setPathState(PathState.SPIKE1_SHOOT);
                }
                break;

            case SPIKE1_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.Spike1ToShoot);
                }
                if (follower.getPose().getX() < (144 - 44.5) || pathTimer.getElapsedTimeSeconds()>4) {
                    setPathState(PathState.SHOOT);
                }
                break;
            case GET_SPIKE3:
                prevState="GetSpike3";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.ShootToSpike3, true);

                }
                if (follower.getPose().getX() > (144 - 29) || pathTimer.getElapsedTimeSeconds()>4) {
                    setPathState(PathState.SPIKE3_SHOOT);
                }
                break;

            case SPIKE3_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.Spike3ToShoot);
                }
                if (follower.getPose().getX() < (144 - 48) || pathTimer.getElapsedTimeSeconds()>4) {
                    setPathState(PathState.SHOOT);
                }
                break;
            case PARK:
                intake.setIntakeState(IntakeA.IntakeState.IDLE);
                outtake.setOuttakeState(OuttakeA.OuttakeState.OFF);
                if (pathTimer.getElapsedTimeSeconds()<0.1) {
                    follower.followPath(paths.Park, true);
                }
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
    public void initialize() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        intake = new IntakeA(hardwareMap);
        outtake = new OuttakeA(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        follower.setPose(new Pose(144 - 18, 120, Math.toRadians(180 - 144)));
        pathState = PathState.PRELOAD;

        Globals.alliance = Alliance.RED;
    }

    @Override
    public void periodic() {
        if(!toggle){
            opModeTimer.resetTimer();
            setPathState(PathState.PRELOAD);
            intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN);
            outtake.setTarget_Vel(1820);
            outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
            toggle = true;
        }
        follower.update();
        statePathUpdate();
        outtake.periodic();
        intake.periodic();

        telemetry.addData("State", pathState);
        //telemetry.addData("Spike Count", spikeCount);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void end() {
        if (intake != null) {
            intake.setIntakeState(IntakeA.IntakeState.IDLE);
            intake.periodic();
        }
        if (outtake != null) {
            outtake.setOuttakeState(OuttakeA.OuttakeState.OFF);
            outtake.periodic();
        }
        Storage.isRed=false;
        Storage.lastPose = follower.getPose();
    }

    @Override
    public void telemetry(Telemetry tele) {

    }
}