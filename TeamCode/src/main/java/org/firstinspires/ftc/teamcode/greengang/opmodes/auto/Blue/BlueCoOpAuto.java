
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

import java.util.Objects;


@Autonomous(name = "BlueCoOpAuto")
public class BlueCoOpAuto extends OpMode {
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
        public Pose shootPose = new Pose(46.500, 97.000, Math.toRadians(142));
        public Pose spike2Entry = new Pose(41.190, 60, Math.toRadians(180));
        public Pose spike2PickUp = new Pose(9.23, 58.4, Math.toRadians(182));
        public Pose gateCycle = new Pose(8.9, 57.6, Math.toRadians(154.2));
        public Pose spike1Entry = new Pose(42, 84, Math.toRadians(180));
        public Pose spike1PickUp = new Pose(13.9, 84, Math.toRadians(181));
        public Pose spike3Entry = new Pose(40.271, 36, Math.toRadians(180));
        public Pose spike3PickUp = new Pose(9, 34.7, Math.toRadians(182));
        public Pose parkPose = new Pose(45, 120, Math.toRadians(180));


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

    private IntakeA intake;
    private OuttakeA outtake;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    private Paths paths;

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
    double start_shoot_time = 0.02;
    int gateCycles = 0;

    private String prevState;
    //private int spikeCount = 0;





    public void statePathUpdate() {
        switch (pathState) {
            case PRELOAD:
                prevState = "Preload";
                if (follower.getPose().getX() < 18.5) {
                    follower.followPath(paths.Preload, true);
                    outtake.setHoodPosition(0.022);
                }
                if (follower.getPose().getX() > 44.6) {
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
                } else if (pathTimer.getElapsedTimeSeconds() >= start_shoot_time + 1.25 && pathTimer.getElapsedTimeSeconds() < start_shoot_time + 1.71) {
                    outtake.setTarget_Vel(1955);
                    outtake.setOuttakeState(OuttakeA.OuttakeState.FIRE);
                    intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN);
                } else if (pathTimer.getElapsedTimeSeconds() >= start_shoot_time + 2.2) {
                    outtake.setOuttakeState(OuttakeA.OuttakeState.REV);
                    if (Objects.equals(prevState, "Preload")) {
                        setPathState(PathState.GET_SPIKE1);
                    }
                    else if (Objects.equals(prevState, "GetSpike1")) {
                        gateCycles = 0;
                        setPathState(PathState.GET_GATE_CYCLE);
                    }
                    else if (Objects.equals(prevState, "GetGateCycle1")) {
                        gateCycles++;

                        if (gateCycles < 2) {
                            setPathState(PathState.GET_GATE_CYCLE);
                        } else {
                            setPathState(PathState.PARK);
                        }
                    }



                }
                break;

            case GET_SPIKE2:
                prevState="GetSpike2";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.ShootToSpike2, true);

                }
                if (follower.getPose().getX() < 10) {
                    setPathState(PathState.SPIKE2_SHOOT);
                }
                break;

            case SPIKE2_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.Spike2ToShoot);
                }
                if (follower.getPose().getY()>89) { //y val could be decreased
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
                    follower.setMaxPower(0.65);
                }
                if ((follower.getPose().getX() < 9.3 && pathTimer.getElapsedTimeSeconds() > 2) || pathTimer.getElapsedTimeSeconds()>4) {
                    follower.setMaxPower(1.0);
                    setPathState(PathState.GATE_CYCLE_SHOOT);
                }
                break;

            case GATE_CYCLE_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.GateCycleToShoot);
                }
                if (follower.getPose().getX()>45.2) {
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
                if (follower.getPose().getX() < 16.7) {
                    follower.setMaxPower(1.0);
                    setPathState(PathState.SPIKE1_SHOOT);
                }
                break;

            case SPIKE1_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.Spike1ToShoot);
                }
                if (follower.getPose().getX()>44.9) {
                    setPathState(PathState.SHOOT);
                }
                break;
            case GET_SPIKE3:
                prevState="GetSpike3";
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.ShootToSpike3, true);

                }
                if (follower.getPose().getX() < 10) {
                    setPathState(PathState.SPIKE3_SHOOT);
                }
                break;

            case SPIKE3_SHOOT:
                if (pathTimer.getElapsedTimeSeconds() < 0.1) {
                    follower.followPath(paths.Spike3ToShoot);
                }
                if (follower.getPose().getX()>46) {
                    setPathState(PathState.SHOOT);
                }
                break;
            case PARK:
                if (pathTimer.getElapsedTimeSeconds()<0.1) {
                    follower.followPath(paths.Park);
                }
                if (follower.getPose().getY()> 116) {
                    setPathState(PathState.END);
                }
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
    public void init() {
        pathTimer = new Timer();
        opModeTimer = new Timer();
        intake = new IntakeA(hardwareMap);
        outtake = new OuttakeA(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        paths = new Paths(follower);
        follower.setPose(new Pose(18, 120, Math.toRadians(144)));
        pathState = PathState.PRELOAD;
    }

    @Override
    public void start() {
        opModeTimer.resetTimer();
        setPathState(PathState.PRELOAD);
        intake.setIntakeState(IntakeA.IntakeState.INTAKE_IN);
        outtake.setTarget_Vel(2050.0);
        outtake.setOuttakeState(OuttakeA.OuttakeState.REV);

        gateCycles = 0;
    }

    @Override
    public void loop() {
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
}