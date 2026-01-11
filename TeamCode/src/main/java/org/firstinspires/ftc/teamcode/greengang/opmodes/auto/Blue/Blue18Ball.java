
package org.firstinspires.ftc.teamcode.greengang.opmodes.auto.Blue;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commonA.intakeA.IntakeA;
import org.firstinspires.ftc.teamcode.commonA.outtakeA.OuttakeA;
import org.firstinspires.ftc.teamcode.greengang.GlobalsStorage.Storage;
import org.firstinspires.ftc.teamcode.greengang.common.util.Alliance;
import org.firstinspires.ftc.teamcode.greengang.common.util.Globals;
import org.firstinspires.ftc.teamcode.greengang.opmodes.GreenLinearOpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Objects;


@Autonomous(name = "Auto Blue 18 Ball", preselectTeleOp = "Tele")
public class Blue18Ball extends GreenLinearOpMode {
    @Override
    public void telemetry(Telemetry tele) {

    }

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
        public Pose startPose = new Pose(18, 120, Math.toRadians(144));
        public Pose shootPose = new Pose(46.500, 97.000, Math.toRadians(143.5));
        public Pose spike2Entry = new Pose(41.190, 60, Math.toRadians(180));
        public Pose spike2PickUp = new Pose(9.32, 58.2, Math.toRadians(182));
        public Pose gateCycle = new Pose(8.9, 57.6, Math.toRadians(154.2));
        public Pose spike1Entry = new Pose(42, 84, Math.toRadians(180));
        public Pose spike1PickUp = new Pose(14.3, 84, Math.toRadians(181));
        public Pose spike3Entry = new Pose(40.271, 36, Math.toRadians(180));
        public Pose spike3PickUp = new Pose(9.5, 34.7, Math.toRadians(182));
        public Pose parkPose = new Pose(21, 100, Math.toRadians(135));


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
                                    new Pose(54.798, 70.446),
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
                                    new Pose(65, 48),
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
                                    new Pose(52.591, gateCycle.getY()),
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
                                    new Pose(52.407, 61.620),
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
                                    new Pose(53.143, 82.399),
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
                                    new Pose(42.661, 86.996),
                                    shootPose
                            )
                    )
                    .setLinearHeadingInterpolation(spike1PickUp.getHeading(), shootPose.getHeading())
                    .build();
            ShootToSpike3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    shootPose,
                                    new Pose(76.312, 35.508),
                                    spike3Entry
                            )
                    )
                    .setLinearHeadingInterpolation(shootPose.getHeading(), spike3Entry.getHeading())
                    .addPath(
                            new BezierLine(spike3Entry, spike3PickUp)
                    )
                    .setLinearHeadingInterpolation(spike3Entry.getHeading(), spike3PickUp.getHeading())
                    .build();



            Spike3ToShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierCurve(
                                    spike3PickUp,
                                    new Pose(65.095, 63.459),
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

    private boolean toggle = false;

    private IntakeA intake;
    private OuttakeA outtake;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Paths paths;
    private double gateCount = 0;

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

    // --- POSES ---
    /*private final Pose startPose = new Pose(18, 120, Math.toRadians(144));
    private final Pose shootPose = new Pose(46.5, 97, Math.toRadians(142));
    private final Pose gateCyclePose = new Pose(9.13, 58.46, Math.toRadians(147.5));

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
*/
    double start_shoot_time = -0.3;

    private String prevState;
    //private int spikeCount = 0;





    public void statePathUpdate() {
        switch (pathState) {
            case PRELOAD:
                prevState = "Preload";
                if (follower.getPose().getX() < 18.5) {
                    follower.followPath(paths.Preload, true);
                    outtake.setHoodPosition(0);
                    outtake.setOuttakeState(OuttakeA.OuttakeState.REV);

                }
                if (follower.getPose().getX() > 36) {
                    setPathState(PathState.SHOOT);
                }
                break;

            case SHOOT:
                if (pathTimer.getElapsedTimeSeconds()<0.25) outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                else outtake.setOuttakeState(OuttakeA.OuttakeState.SPIN_UP);
                if (pathTimer.getElapsedTimeSeconds()< 0.34 && Objects.equals(prevState, "Preload")) {
                    outtake.setTarget_Vel(1600);
                }
                else if (Objects.equals(prevState, "Preload")) outtake.setTarget_Vel(1580);
                else outtake.setTarget_Vel(Storage.autoAimFlywheelPow);


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
                if (follower.getPose().getX() < 10 || pathTimer.getElapsedTimeSeconds()>4) {
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
                if (follower.getPose().getX()<33 && follower.getPose().getX()>9.3) {
                    follower.setMaxPower(0.7);
                }
                if (pathTimer.getElapsedTimeSeconds()>3.5 && gateCount==1) {
                    follower.setMaxPower(1.0);
                    gateCount+=1;
                    setPathState(PathState.GATE_CYCLE_SHOOT);
                }
                if (pathTimer.getElapsedTimeSeconds()>3.5 && gateCount==0|| pathTimer.getElapsedTimeSeconds()>7) {
                    follower.setMaxPower(1.0);
                    gateCount+=1;
                    setPathState(PathState.GATE_CYCLE_SHOOT);
                }
                break;

            case GATE_CYCLE_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.GateCycleToShoot);
                }
                if (follower.getPose().getX()>45.7|| pathTimer.getElapsedTimeSeconds()>5) {
                    setPathState(PathState.SHOOT);
                }
                break;
            case GET_SPIKE1:
                prevState="GetSpike1";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.ShootToSpike1, true);

                }
                if (follower.getPose().getX() < 43.5) {
                    follower.setMaxPower(0.65);
                }
                if (follower.getPose().getX() < 16.7|| pathTimer.getElapsedTimeSeconds()>4) {
                    follower.setMaxPower(1.0);
                    setPathState(PathState.SPIKE1_SHOOT);
                }
                break;

            case SPIKE1_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.Spike1ToShoot);
                }
                if (follower.getPose().getX()>44.5|| pathTimer.getElapsedTimeSeconds()>4) {
                    setPathState(PathState.SHOOT);
                }
                break;
            case GET_SPIKE3:
                prevState="GetSpike3";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.ShootToSpike3, true);

                }
                if (follower.getPose().getX() < 10|| pathTimer.getElapsedTimeSeconds()>4) {
                    setPathState(PathState.SPIKE3_SHOOT);
                }
                break;

            case SPIKE3_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.Spike3ToShoot);
                }
                if (follower.getPose().getX()>46|| pathTimer.getElapsedTimeSeconds()>4) {
                    setPathState(PathState.SHOOT);
                }
                break;
            case PARK:
                intake.setIntakeState(IntakeA.IntakeState.IDLE);
                outtake.setOuttakeState(OuttakeA.OuttakeState.OFF);
                if (pathTimer.getElapsedTimeSeconds()<0.1) {
                    follower.followPath(paths.Park, true);
                }
                /*if (follower.getPose().getY()> 116) {
                    setPathState(PathState.END);
                }*/
                break;
            case END:

                intake.setIntakeState(IntakeA.IntakeState.IDLE);
                outtake.setOuttakeState(OuttakeA.OuttakeState.OFF);
                //requestOpModeStop();
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
        follower.setPose(new Pose(18, 120, Math.toRadians(144)));
        pathState = PathState.PRELOAD;

        Globals.alliance = Alliance.BLUE;
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

        Storage.lastPose = follower.getPose();
    }
}